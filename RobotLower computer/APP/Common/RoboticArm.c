#include "RoboticArm.h"

/* 根据三电机当前反馈转角,按几何关系正解出末端坐标与杆自转角度,写回arm对应字段 */
static void RoboticArmUpdateStateFromFeedback(RoboticArm_TypeDef *arm)
{
    float height = ROBOTICARM_LIFT_K * arm->lift_motor.feedback.position + ROBOTICARM_LIFT_THRESHOLD;
    float distance = ROBOTICARM_FORWARD_K * arm->forward_motor.feedback.position + ROBOTICARM_FORWARD_THRESHOLD;

    arm->end_x = ROBOTICARM_BASE_X + distance;
    arm->end_y = ROBOTICARM_BASE_Y + ROBOTICARM_ROD_LENGTH;
    arm->end_z = height + ROBOTICARM_END_Z_OFFSET;
    arm->rod_rotation = arm->rotate_motor.feedback.position; /* 电机转角与杆自转角度直接相等 */
}

void RoboticArmInit(RoboticArm_TypeDef *arm,
                     FDCAN_HandleTypeDef *lift_FDCAN_Handle, uint8_t lift_id,
                     UART_HandleTypeDef *forward_huart, uint8_t forward_id,
                     UART_HandleTypeDef *rotate_huart, uint8_t rotate_id)
{
    J60MotorInit(&arm->lift_motor, lift_FDCAN_Handle, lift_id);
    GOM8010MotorInit(&arm->forward_motor, forward_id, forward_huart);
    GOM8010MotorInit(&arm->rotate_motor, rotate_id, rotate_huart);

    RoboticArmUpdateStateFromFeedback(arm);
}

void RoboticArmSetEndPosition(RoboticArm_TypeDef *arm, float end_x_target, float end_z_target)
{
    float distance_target = end_x_target - ROBOTICARM_BASE_X;
    float height_target = end_z_target - ROBOTICARM_END_Z_OFFSET;

    float theta_forward_target = (distance_target - ROBOTICARM_FORWARD_THRESHOLD) / ROBOTICARM_FORWARD_K;
    float theta_lift_target = (height_target - ROBOTICARM_LIFT_THRESHOLD) / ROBOTICARM_LIFT_K;

    J60MotorSetTarget(&arm->lift_motor, theta_lift_target);
    GOM8010MotorSetTarget(&arm->forward_motor, theta_forward_target);
}

void RoboticArmSetRodRotation(RoboticArm_TypeDef *arm, float rotation_target)
{
    GOM8010MotorSetTarget(&arm->rotate_motor, rotation_target); /* 电机转角与杆自转角度直接相等,无需换算 */
}

void RoboticArmUpdate(RoboticArm_TypeDef *arm)
{
    J60MotorUpdate(&arm->lift_motor);
    GOM8010MotorUpdate(&arm->forward_motor);
    GOM8010MotorUpdate(&arm->rotate_motor);

    RoboticArmUpdateStateFromFeedback(arm);
}

void RoboticArmEnable(RoboticArm_TypeDef *arm)
{
    J60MotorEnable(&arm->lift_motor);
}

void RoboticArmDisable(RoboticArm_TypeDef *arm)
{
    J60MotorDisable(&arm->lift_motor);
}
