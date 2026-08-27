#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "bsp_config.h"

#define SERVO_MAX_ANGLE BSP_PI
#define SERVO_MIN_ANGLE 0.0f
#define SERVO_TRANS_PARAMETER (1390.0f / BSP_PI)

/*
 * 舵机 PWM 脉宽标定：
 * 当前机械安装方向与原始映射相反，故软件角度越大，输出脉宽越小。
 *   0 rad  -> 2390 us
 *   PI rad -> 1000 us
 * 这样 Flip 下发 PI rad 时，舵机将转向原先相反的物理端点。
 */
#define SERVO_MIN_PULSE_US            1000U
#define SERVO_ZERO_ANGLE_PULSE_US     (SERVO_MIN_PULSE_US + 1390U)
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
