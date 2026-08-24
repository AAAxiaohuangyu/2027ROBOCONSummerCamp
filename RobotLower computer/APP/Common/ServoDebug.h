#ifndef __SERVO_DEBUG_H__
#define __SERVO_DEBUG_H__

#include "stm32h7xx_hal.h"

/* Servo signal: JP6 pin 3, PB5 (AF2) -> TIM3_CH2. JP6 pin 2 is GND and pin 1 is 5 V. */
HAL_StatusTypeDef ServoDebugInit(void);

/* Call periodically from a task. Sweeps 0, 90 and 180 degrees every 2 seconds. */
void ServoDebugUpdate(void);

#endif
