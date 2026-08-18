#ifndef __MOTOR_GOTO_ZERO_H
#define __MOTOR_GOTO_ZERO_H

#include "J60.h"
#include "GO_M8010.h"

/* 下发J60电机零位目标 */
void J60MotorGotoZero(J60Motor_TypeDef *motor);

/* 下发GO-M8010电机输出端零位目标 */
void GOM8010MotorGotoZero(GOM8010Motor_TypeDef *motor);


#endif
