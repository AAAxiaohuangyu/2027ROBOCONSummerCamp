#include "J60.h"
#include <string.h>

static uint16_t J60FloatToUint(float value, float min, float max, uint8_t bits)
{
    float ratio = (value - min) / (max - min);
    uint32_t limit = (1UL << bits) - 1UL;

    if (ratio < 0.0f)
        ratio = 0.0f;
    if (ratio > 1.0f)
        ratio = 1.0f;

    return (uint16_t)(ratio * (float)limit);
}

static float J60UintToFloat(uint32_t value, float min, float max, uint8_t bits)
{
    uint32_t limit = (1UL << bits) - 1UL;

    return (float)value * (max - min) / (float)limit + min;
}

static uint16_t J60CommandId(uint8_t id, uint8_t command, uint8_t response)
{
    return (uint16_t)id |
           ((uint16_t)response << J60_CAN_ID_RESPONSE_SHIFT) |
           ((uint16_t)command << J60_CAN_ID_COMMAND_SHIFT);
}

/* 按协议打包8字节控制帧:位置/速度取自规划结果,kp/kd取自控制参数,torque前馈恒为0 */
static void J60PackControlFrame(J60Control_TypeDef *control, uint8_t *data)
{
    uint64_t frame = 0;

    frame |= (uint64_t)J60FloatToUint(control->position, J60_POSITION_MIN, J60_POSITION_MAX, J60_CONTROL_POSITION_BITS);
    frame |= (uint64_t)J60FloatToUint(control->velocity, J60_VELOCITY_MIN, J60_VELOCITY_MAX, J60_CONTROL_VELOCITY_BITS) << J60_CONTROL_VELOCITY_SHIFT;
    frame |= (uint64_t)J60FloatToUint(control->kp, J60_KP_MIN, J60_KP_MAX, J60_CONTROL_KP_BITS) << J60_CONTROL_KP_SHIFT;
    frame |= (uint64_t)J60FloatToUint(control->kd, J60_KD_MIN, J60_KD_MAX, J60_CONTROL_KD_BITS) << J60_CONTROL_KD_SHIFT;
    frame |= (uint64_t)J60FloatToUint(control->torque, J60_TORQUE_MIN, J60_TORQUE_MAX, J60_CONTROL_TORQUE_BITS) << J60_CONTROL_TORQUE_SHIFT;

    for (uint8_t i = 0; i < J60_CAN_FRAME_LENGTH; i++)
        data[i] = (uint8_t)(frame >> (8U * i));
}

/* 解析8字节反馈帧:位置/速度/扭矩/温度按协议位域提取 */
static void J60ParseFeedbackFrame(J60Feedback_TypeDef *feedback, const uint8_t *rx_data)
{
    uint64_t frame;

    memcpy(&frame, rx_data, sizeof(frame));

    feedback->position = J60UintToFloat(frame & J60_FEEDBACK_POSITION_MASK, J60_POSITION_MIN, J60_POSITION_MAX, J60_FEEDBACK_POSITION_BITS);
    feedback->velocity = J60UintToFloat((frame >> J60_FEEDBACK_VELOCITY_SHIFT) & J60_FEEDBACK_POSITION_MASK, J60_VELOCITY_MIN, J60_VELOCITY_MAX, J60_FEEDBACK_VELOCITY_BITS);
    feedback->torque = J60UintToFloat((frame >> J60_FEEDBACK_TORQUE_SHIFT) & J60_FEEDBACK_TORQUE_MASK, J60_TORQUE_MIN, J60_TORQUE_MAX, J60_FEEDBACK_TORQUE_BITS);
    feedback->temperature_is_motor = (uint8_t)((frame >> J60_FEEDBACK_TEMP_SENSOR_SHIFT) & J60_FEEDBACK_TEMP_SENSOR_MASK);
    feedback->temperature = (float)((frame >> J60_FEEDBACK_TEMP_SHIFT) & J60_FEEDBACK_TEMP_MASK) * J60_TEMPERATURE_SCALE / J60_TEMPERATURE_RAW_MAX + J60_TEMPERATURE_OFFSET;
    feedback->update_cnt++;
}

static void J60MotorSendControl(J60Motor_TypeDef *motor)
{
    uint8_t data[J60_CAN_FRAME_LENGTH];

    J60PackControlFrame(&motor->control, data);
    FDCANSendStandard(motor->FDCAN_Handle, J60CommandId(motor->id, J60_CMD_CONTROL, J60_RESPONSE_REQUEST), data, J60_CAN_FRAME_LENGTH);
}

void J60MotorInit(J60Motor_TypeDef *motor, FDCAN_HandleTypeDef *FDCAN_Handle, uint8_t id)
{
    memset(&motor->control, 0, sizeof(motor->control));
    memset(&motor->feedback, 0, sizeof(motor->feedback));

    motor->id = id & 0x0FU;
    motor->FDCAN_Handle = FDCAN_Handle;

    SpeedPlanInit(&motor->control.plan, J60_POS_CTRL_A_MAX, J60_POS_CTRL_V_MAX, J60_POS_CTRL_J);
    motor->control.kp = J60_POS_CTRL_KP;
    motor->control.kd = J60_POS_CTRL_KD;
    motor->control.position_target = 0.0f;
}

void J60MotorSetTarget(J60Motor_TypeDef *motor, float position_target)
{
    motor->control.position_target = position_target;
    motor->control.plan.state = init; /* 触发(重新)规划,运动中调用即为打断 */
}

void J60MotorSetTorqueFeedforward(J60Motor_TypeDef *motor, float torque_feedforward)
{
    motor->control.torque_feedforward = torque_feedforward;
}

void J60MotorEnable(J60Motor_TypeDef *motor)
{
    uint8_t data[J60_CAN_FRAME_LENGTH] = {0};

    FDCANSendStandard(motor->FDCAN_Handle, J60CommandId(motor->id, J60_CMD_ENABLE, J60_RESPONSE_REQUEST), data, 0U);
}

void J60MotorDisable(J60Motor_TypeDef *motor)
{
    uint8_t data[J60_CAN_FRAME_LENGTH] = {0};

    FDCANSendStandard(motor->FDCAN_Handle, J60CommandId(motor->id, J60_CMD_DISABLE, J60_RESPONSE_REQUEST), data, 0U);
}

uint8_t J60MotorParseFeedback(J60Motor_TypeDef *motor, uint32_t std_id, const uint8_t *rx_data)
{
    if (std_id != J60CommandId(motor->id, J60_CMD_CONTROL, J60_RESPONSE_FEEDBACK))
        return 0U;

    J60ParseFeedbackFrame(&motor->feedback, rx_data);
    return 1U;
}

void J60MotorUpdate(J60Motor_TypeDef *motor)
{
    J60Control_TypeDef *control = &motor->control;
    float position_actual = motor->feedback.position; /* 电机CAN反馈的真实位置 */

    SpeedPlanUpdate(&control->plan, position_actual, control->position_target);

    /* 力位速混合控制:torque前馈由上层指定,叠加到kp/kd跟踪规划轨迹给出的position/speed上 */
    control->torque = control->torque_feedforward;
    control->position = control->plan.position_initial + control->plan.direction_flag * control->plan.s;
    control->velocity = control->plan.v * control->plan.direction_flag;

    J60MotorSendControl(motor);
}
