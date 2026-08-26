#include "GO_M8010.h"
#include <string.h>

#define SATURATE(_IN, _MIN, _MAX) \
    do {                           \
        if ((_IN) <= (_MIN))      \
            (_IN) = (_MIN);       \
        else if ((_IN) >= (_MAX)) \
            (_IN) = (_MAX);       \
    } while (0)

/* CRC-16/CCITT(反射多项式0x8408,对应正规多项式0x1021)查找表,与GO-M8010-6协议一致 */
static const uint16_t crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

static uint16_t crc_ccitt_byte(uint16_t crc, uint8_t byte)
{
    return (crc >> 8) ^ crc_ccitt_table[(crc ^ (uint16_t)byte) & 0xFFU];
}

static uint16_t crc_ccitt(uint16_t crc, const uint8_t *buffer, size_t len)
{
    while (len-- > 0U)
        crc = crc_ccitt_byte(crc, *buffer++);
    return crc;
}

static void go_m8010_put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void go_m8010_put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static uint16_t go_m8010_get_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static uint32_t go_m8010_get_u32_le(const uint8_t *src)
{
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

/* 输出端->转子侧:扭矩经减速器放大,按减速比乘;速度/位置经减速器缩小,按减速比除 */
static float go_m8010_torque_output_to_rotor(float torque_output)
{
    return torque_output / GO_M8010_REDUCTION_RATIO;
}

static float go_m8010_speed_output_to_rotor(float speed_output)
{
    return speed_output * GO_M8010_REDUCTION_RATIO;
}

static float go_m8010_position_output_to_rotor(float position_output)
{
    return position_output * GO_M8010_REDUCTION_RATIO;
}

/* 转子侧->输出端:与上面互为逆运算 */
static float go_m8010_torque_rotor_to_output(float torque_rotor)
{
    return torque_rotor * GO_M8010_REDUCTION_RATIO;
}

static float go_m8010_speed_rotor_to_output(float speed_rotor)
{
    return speed_rotor / GO_M8010_REDUCTION_RATIO;
}

static float go_m8010_position_rotor_to_output(float position_rotor)
{
    return position_rotor / GO_M8010_REDUCTION_RATIO;
}

/* 按GO-M8010-6协议打包17字节控制帧,限幅后转换为定点数,不回写调用者原始指令 */
static void GOM8010PackControlFrame(GOM8010Motor_TypeDef *motor)
{
    GOM8010Control_TypeDef *control = &motor->control;
    uint8_t *frame = control->packet.bytes;
    uint8_t id = motor->id;
    uint8_t mode = control->mode;
    uint8_t timeout = control->timeout;
    float torque = go_m8010_torque_output_to_rotor(control->torque);
    float speed = go_m8010_speed_output_to_rotor(control->speed);
    /* control->position是软件零点坐标系(与feedback->position同系),打包发送前需加回position_offset
       换算回电机原始坐标系,否则电机实际运动位置会偏差一个position_offset */
    float position = go_m8010_position_output_to_rotor(control->position + motor->feedback.position_offset);
    float kp = control->kp;
    float kd = control->kd;
    int16_t torque_raw;
    int16_t speed_raw;
    int32_t position_raw;
    int16_t kp_raw;
    int16_t kd_raw;
    uint16_t crc;

    if (id > 15U)      /* id占4bit */
        id = 15U;
    if (mode > 2U)     /* 0=锁定,1=FOC闭环(力位速混合控制,常用模式),2=编码器校准 */
        mode = 2U;
    if (timeout > 1U)
        timeout = 1U;
    SATURATE(kp, 0.0f, 25.599f);              /* kp/kd满量程25.6,对应16bit定点 */
    SATURATE(kd, 0.0f, 25.599f);
    SATURATE(torque, -4.0f, 4.0f);            /* 转子侧额定最大转矩±4N*m(已由输出端换算而来) */
    SATURATE(speed, -804.0f, 804.0f);         /* 转子侧最大转速±804rad/s,由16bit定点满量程反推 */
    SATURATE(position, -411774.0f, 411774.0f); /* 转子侧位置范围,由32bit定点满量程反推 */

    torque_raw = (int16_t)(torque * 256.0f);
    speed_raw = (int16_t)(speed / PI2 * 256.0f);
    position_raw = (int32_t)(position / PI2 * 32768.0f);
    kp_raw = (int16_t)(kp / 25.6f * 32768.0f);
    kd_raw = (int16_t)(kd / 25.6f * 32768.0f);

    frame[0] = 0xFEU; /* 控制帧帧头 */
    frame[1] = 0xEEU;
    frame[2] = (uint8_t)((id & 0x0FU) |
                         ((mode & 0x07U) << 4) |
                         ((timeout & 0x01U) << 7));
    go_m8010_put_u16_le(&frame[3], (uint16_t)torque_raw);
    go_m8010_put_u16_le(&frame[5], (uint16_t)speed_raw);
    go_m8010_put_u32_le(&frame[7], (uint32_t)position_raw);
    go_m8010_put_u16_le(&frame[11], (uint16_t)kp_raw);
    go_m8010_put_u16_le(&frame[13], (uint16_t)kd_raw);

    crc = crc_ccitt(0U, frame, 15U); /* CRC覆盖帧头+数据共15字节 */
    go_m8010_put_u16_le(&frame[15], crc);
}

/* 解析16字节反馈帧:校验帧头与CRC后提取torque/speed/position及状态量;第一帧的原始position作为
   软件零点(position_offset),之后每帧position均减去该offset */
static void GOM8010ParseFeedbackFrame(GOM8010Feedback_TypeDef *feedback)
{
    const uint8_t *frame = feedback->packet.bytes;
    uint16_t received_crc;
    uint16_t status_force;
    int16_t torque_raw;
    int16_t speed_raw;
    int32_t position_raw;
    float position_output;

    feedback->valid = 0U;
    if ((frame[0] != 0xFDU) || (frame[1] != 0xEEU)) /* 反馈帧帧头 */
    {
        feedback->bad_msg++;
        return;
    }

    feedback->calc_crc = crc_ccitt(0U, frame, 14U); /* CRC覆盖前14字节 */
    received_crc = go_m8010_get_u16_le(&frame[14]);
    if (received_crc != feedback->calc_crc)
    {
        feedback->bad_msg++;
        return;
    }

    torque_raw = (int16_t)go_m8010_get_u16_le(&frame[3]);
    speed_raw = (int16_t)go_m8010_get_u16_le(&frame[5]);
    position_raw = (int32_t)go_m8010_get_u32_le(&frame[7]);
    status_force = go_m8010_get_u16_le(&frame[12]);

    feedback->mode = (frame[2] >> 4) & 0x07U;
    feedback->timeout = (frame[2] >> 7) & 0x01U;
    feedback->temp = (int8_t)frame[11];
    feedback->error = status_force & 0x07U;          /* 低3位:故障码 */
    feedback->force = (status_force >> 3) & 0x0FFFU; /* 高12位:末端力传感器读数 */
    feedback->torque = go_m8010_torque_rotor_to_output((float)torque_raw / 256.0f);
    feedback->speed = go_m8010_speed_rotor_to_output((float)speed_raw / 256.0f * PI2);

    position_output = go_m8010_position_rotor_to_output((float)position_raw / 32768.0f * PI2);
    if (feedback->update_cnt == 0U)
        feedback->position_offset = position_output;

    feedback->position = position_output - feedback->position_offset;
    feedback->update_cnt++;
    feedback->valid = 1U;
}

static void GOM8010MotorSendControl(GOM8010Motor_TypeDef *motor)
{
    GOM8010PackControlFrame(motor);
    HAL_UART_Transmit_DMA(motor->huart, motor->control.packet.bytes, GO_M8010_CONTROL_FRAME_SIZE);
}

static void GOM8010MotorParseFeedback(GOM8010Motor_TypeDef *motor, uint16_t size)
{
    if (size != GO_M8010_FEEDBACK_FRAME_SIZE)
    {
        motor->feedback.valid = 0U;
        motor->feedback.bad_msg++;
        return;
    }

    GOM8010ParseFeedbackFrame(&motor->feedback);
}

/* 尝试为group内index号电机发起一次请求(打包控制帧+发送+挂起本电机的接收缓冲区)。同一总线
   上还有其他电机的应答未到/未超时,或刚轮到过本电机、总线上还有别的电机没轮到时,本次跳过不
   发送,返回0;发送成功返回1 */
static uint8_t GOM8010GroupRequest(GOM8010Group_TypeDef *group, uint8_t index)
{
    GOM8010Motor_TypeDef *motor = &group->motors[index];
    GOM8010BusArbiter_TypeDef *bus = &group->arbiter;
    uint32_t now;

    if (motor->huart == NULL)
        return 0U;

    now = HAL_GetTick();

    if (bus->pending_motor != NULL)
    {
        if ((now - bus->request_tick) < GO_M8010_BUS_TIMEOUT_MS)
            return 0U; /* 总线忙,别的电机应答还没到/没超时 */

        /* 上一笔请求应答超时,视为丢失,释放总线让给下一个电机 */
        bus->pending_motor->feedback.valid = 0U;
        bus->pending_motor->feedback.bad_msg++;
        bus->pending_motor = NULL;
    }

    if (bus->motor_count > 1U && bus->last_motor == motor)
        return 0U; /* 总线上还有别的电机没轮到,先让给它,避免本电机连续独占总线 */

    GOM8010MotorSendControl(motor);
    /* 反馈帧长度固定已知,改用定长DMA接收(HAL_UART_Receive_DMA),避免电机内部分两段发送时
       中间的短暂间隙被误判为空闲线(IDLE)从而提前把一帧切断——凑满GO_M8010_FEEDBACK_FRAME_SIZE
       字节才会触发HAL_UART_RxCpltCallback,不受发送节奏影响 */
    if (HAL_UART_Receive_DMA(motor->huart, motor->feedback.packet.bytes, GO_M8010_FEEDBACK_FRAME_SIZE) != HAL_OK)
        return 0U;

    bus->pending_motor = motor;
    bus->last_motor = motor;
    bus->request_tick = now;
    return 1U;
}

void GOM8010GroupInit(GOM8010Group_TypeDef *group)
{
    memset(group, 0, sizeof(*group));
}

uint8_t GOM8010GroupAddMotor(GOM8010Group_TypeDef *group, uint8_t id, UART_HandleTypeDef *huart)
{
    uint8_t index;
    GOM8010Motor_TypeDef *motor;

    if (group->motor_count >= GOM8010_GROUP_MAX_MOTORS)
        return 0xFFU;

    index = group->motor_count;
    motor = &group->motors[index];

    memset(motor, 0, sizeof(*motor));
    motor->id = id;
    motor->huart = huart;
    motor->control.mode = 1U; /* 默认FOC闭环(力位速混合控制),GO电机通常无需切换其他模式 */

    motor->control.position_param.a_max = GO_M8010_POS_CTRL_A_MAX;
    motor->control.position_param.v_max = GO_M8010_POS_CTRL_V_MAX;
    motor->control.position_param.j = GO_M8010_POS_CTRL_J;
    motor->control.position_param.kp = GO_M8010_POS_CTRL_KP;
    motor->control.position_param.kd = GO_M8010_POS_CTRL_KD;
    motor->control.velocity_param.kp = GO_M8010_VEL_CTRL_KP;
    motor->control.velocity_param.kd = GO_M8010_VEL_CTRL_KD;

    SpeedPlanInit(&motor->control.plan, motor->control.position_param.a_max, motor->control.position_param.v_max, motor->control.position_param.j, 0.0f);
    motor->control.ctrl_mode = GOM8010_CTRL_MODE_POSITION;
    motor->control.kp = motor->control.position_param.kp;
    motor->control.kd = motor->control.position_param.kd;
    motor->control.position_target = 0.0f;

    group->arbiter.motor_count++;
    group->motor_count++;
    return index;
}

void GOM8010GroupSetTarget(GOM8010Group_TypeDef *group, uint8_t index, float position_target)
{
    GOM8010Control_TypeDef *control = &group->motors[index].control;

    control->ctrl_mode = GOM8010_CTRL_MODE_POSITION;
    control->position_target = position_target;
    control->plan.state = init; /* 触发(重新)规划,运动中调用即为打断 */
}

void GOM8010GroupSetVelocityTarget(GOM8010Group_TypeDef *group, uint8_t index, float velocity_target)
{
    GOM8010Control_TypeDef *control = &group->motors[index].control;

    control->ctrl_mode = GOM8010_CTRL_MODE_VELOCITY;
    control->velocity_target = velocity_target;
}

void GOM8010GroupSetTorqueFeedforward(GOM8010Group_TypeDef *group, uint8_t index, float torque_feedforward)
{
    group->motors[index].control.torque_feedforward = torque_feedforward;
}

void GOM8010GroupUpdate(GOM8010Group_TypeDef *group)
{
    uint8_t i;

    for (i = 0U; i < group->motor_count; i++)
    {
        GOM8010Motor_TypeDef *motor = &group->motors[i];
        GOM8010Control_TypeDef *control = &motor->control;
        float position_actual = motor->feedback.position; /* 电机RS485反馈的真实位置 */

        /* 力位速混合控制:torque前馈由上层指定,叠加到kp/kd跟踪输出上 */
        control->mode = 1u; /* FOC闭环,力位速混合控制(硬件模式,与ctrl_mode无关) */
        control->timeout = 0u;
        control->torque = control->torque_feedforward;

        if (control->ctrl_mode == GOM8010_CTRL_MODE_VELOCITY)
        {
            /* 定速模式:不做位置规划,kp=0仅由kd跟踪目标速度;position取反馈实际位置,kp=0时不影响输出扭矩 */
            control->kp = control->velocity_param.kp;
            control->kd = control->velocity_param.kd;
            control->position = position_actual;
            control->speed = control->velocity_target;
        }
        else
        {
            SpeedPlanUpdate(&control->plan, position_actual, control->position_target);

            control->kp = control->position_param.kp;
            control->kd = control->position_param.kd;
            control->position = control->plan.position_initial + control->plan.direction_flag * control->plan.s;
            control->speed = control->plan.v * control->plan.direction_flag;
        }

        /* 半双工RS485总线上可能与其他电机共享huart,总线忙时本周期不发送,下一次空闲即发最新目标 */
        GOM8010GroupRequest(group, i);
    }
}

void GOM8010GroupRxEvent(GOM8010Group_TypeDef *group, UART_HandleTypeDef *huart, uint16_t size)
{
    GOM8010BusArbiter_TypeDef *bus = &group->arbiter;

    if (huart == NULL || bus->pending_motor == NULL || bus->pending_motor->huart == NULL ||
        bus->pending_motor->huart->Instance != huart->Instance)
        return; /* 不是本组总线的事件,或总线上没有电机在等应答(意外/迟到事件),丢弃 */

    GOM8010MotorParseFeedback(bus->pending_motor, size);
    bus->pending_motor = NULL; /* 释放总线,供下一个电机的请求使用 */
}

void GOM8010GroupRxErrorEvent(GOM8010Group_TypeDef *group, UART_HandleTypeDef *huart)
{
    GOM8010BusArbiter_TypeDef *bus = &group->arbiter;

    if (huart == NULL || bus->pending_motor == NULL || bus->pending_motor->huart == NULL ||
        bus->pending_motor->huart->Instance != huart->Instance)
        return; /* 不是本组总线的事件,或总线上没有电机在等应答(意外事件),丢弃 */

    /* DMA接收模式下,溢出/帧/噪声等错误均被HAL当作阻塞错误处理:HAL_UART_IRQHandler内部已经
       中止了本笔DMA接收并将RxState还原为READY(见UART_EndRxTransfer+HAL_DMA_Abort_IT),故此处
       不需要重新挂起接收——只需提前释放总线,下一次GOM8010GroupUpdate即会为轮到的电机重新发起
       请求并挂起接收。不在此提前释放的话,也会在GO_M8010_BUS_TIMEOUT_MS后被GOM8010GroupRequest
       的超时逻辑释放,但会让总线上的其他电机多等这段时间 */
    bus->pending_motor->feedback.valid = 0U;
    bus->pending_motor->feedback.bad_msg++;
    bus->pending_motor = NULL;
}
