#include "motor_goto_zero.h"

void J60MotorGotoZero(J60Motor_TypeDef *motor)
{
    J60MotorSetTorqueFeedforward(motor, 0.0f);
    J60MotorSetTarget(motor, 0.0f);
}

void GOM8010MotorGotoZero(GOM8010Motor_TypeDef *motor)
{
    GOM8010MotorSetTorqueFeedforward(motor, 0.0f);
    GOM8010MotorSetTarget(motor, 0.0f);
}
