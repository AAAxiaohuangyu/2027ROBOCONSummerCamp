#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "bsp_config.h"

#define SERVO_MAX_ANGLE BSP_PI
#define SERVO_MIN_ANGLE 0.0f
#define SERVO_TRANS_PARAMETER (2000.0f / BSP_PI)

/* 舵机上电时的初始脉宽，当前机械标定为 500 us。 */
#define SERVO_INITIAL_PULSE_US       500U
#define SERVO_CONTROL_PERIOD 5U

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float angle;
} Servo_TypeDef;

void ServoInit(Servo_TypeDef *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void ServoAngleUpdate(Servo_TypeDef *servo);

#endif
