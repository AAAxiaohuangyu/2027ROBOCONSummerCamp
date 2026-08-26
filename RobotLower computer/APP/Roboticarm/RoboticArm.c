#include "RoboticArm.h"
#include "cmsis_os2.h"

/* 根据升降/前后两电机当前反馈转角,按几何关系正解出末端坐标,并将舵机当前指令角度
   同步为杆自转角度,写回arm对应字段 */
static void
RoboticArmUpdateStateFromFeedback(RoboticArm_TypeDef *arm)
{
    const GOM8010Feedback_TypeDef *forward_feedback = &arm->go_motors.motors[ROBOTICARM_GO_FORWARD].feedback;
    float height = ROBOTICARM_LIFT_K * arm->lift_motor.feedback.position;
    float distance = ROBOTICARM_FORWARD_K * forward_feedback->position;

    arm->end_x = distance;
    arm->end_y = 0.0f;
    arm->end_z = height;
    arm->rod_rotation = arm->rotate_servo.angle; /* 舵机开环,指令角度即当前角度 */
}

void RoboticArmInit(RoboticArm_TypeDef *arm,
                    FDCAN_HandleTypeDef *lift_FDCAN_Handle, uint8_t lift_id,
                    UART_HandleTypeDef *forward_huart, uint8_t forward_id,TIM_HandleTypeDef *htim,uint32_t channel)
{
    J60MotorInit(&arm->lift_motor, lift_FDCAN_Handle, lift_id);

    GOM8010GroupInit(&arm->go_motors);
    GOM8010GroupAddMotor(&arm->go_motors, forward_id, forward_huart);

    ServoInit(&arm->rotate_servo, htim, channel);

    RoboticArmUpdateStateFromFeedback(arm);
}

void RoboticArmSetEndPosition(RoboticArm_TypeDef *arm, float end_x_target, float end_z_target,
                              float lift_torque_feedforward)
{
    float distance_target = end_x_target;
    float height_target = end_z_target;

    float theta_forward_target = distance_target / ROBOTICARM_FORWARD_K;
    float theta_lift_target = height_target / ROBOTICARM_LIFT_K;

    J60MotorSetTarget(&arm->lift_motor, theta_lift_target);
    J60MotorSetTorqueFeedforward(&arm->lift_motor, lift_torque_feedforward);
    GOM8010GroupSetTarget(&arm->go_motors, ROBOTICARM_GO_FORWARD, theta_forward_target);
}

void RoboticArmSetRodRotation(RoboticArm_TypeDef *arm, float rotation_target)
{
    arm->rotate_servo.angle = rotation_target; /* 开环舵机,RobotServoUpdateTask按该角度持续输出PWM */
}

void RoboticArmUpdate(RoboticArm_TypeDef *arm)
{
    while (1)
    {
        J60MotorUpdate(&arm->lift_motor);
        GOM8010GroupUpdate(&arm->go_motors);
        ServoAngleUpdate(&arm->rotate_servo);

        RoboticArmUpdateStateFromFeedback(arm);
        osDelay(ROBOTICARM_CONTROL_PERIOD_MS);
    }
}

void RoboticArmEnable(RoboticArm_TypeDef *arm)
{
    J60MotorEnable(&arm->lift_motor);
}

void RoboticArmDisable(RoboticArm_TypeDef *arm)
{
    J60MotorDisable(&arm->lift_motor);
}

uint8_t PositionReached(RoboticArm_TypeDef *arm,
                        float target_x,
                        float target_z, float x_position_tolerance, float z_position_tolrance)
{
    /* 使用末端反馈坐标判断 x/z 目标是否到位。 */
    float dx = arm->end_x - target_x;
    float dz = arm->end_z - target_z;

    if (dx < 0.0f)
    {
        dx = -dx;
    }

    if (dz < 0.0f)
    {
        dz = -dz;
    }

    return (dx <= x_position_tolerance &&
            dz <= z_position_tolrance);
}

uint8_t RotationReached(RoboticArm_TypeDef *arm,
                        float target_rotation, float rotation_tolerance)
{
    float error = arm->rod_rotation - target_rotation;

    if (error < 0.0f)
    {
        error = -error;
    }

    return (error <= rotation_tolerance);
}
