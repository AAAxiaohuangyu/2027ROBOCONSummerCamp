#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>

/*
 * PWM timer and GPIO are configured by CubeMX.  ServoInit only starts the
 * selected PWM channel and converts pulse widths to the timer compare value.
 * Configure the timer period to the desired PWM period (typically 20 ms).
 */
#define SERVO_DEFAULT_MIN_PULSE_US  500U
#define SERVO_DEFAULT_MAX_PULSE_US  2500U
#define SERVO_DEFAULT_ANGLE_MAX     180U

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t timer_tick_hz; /* PWM timer counter frequency, e.g. 1000000 for 1 tick/us. */
    uint16_t pulse_min_us;
    uint16_t pulse_max_us;
    uint16_t pulse_us;
} Servo_TypeDef;

/*
 * Bind a servo to a CubeMX-configured PWM channel and start output.
 * The configured timer auto-reload value must be longer than pulse_max_us.
 */
HAL_StatusTypeDef ServoInit(Servo_TypeDef *servo, TIM_HandleTypeDef *htim,
                            uint32_t channel, uint32_t timer_tick_hz,
                            uint16_t pulse_min_us, uint16_t pulse_max_us,
                            uint16_t initial_pulse_us);

/* Set pulse width in microseconds. Values outside the configured range are clamped. */
void ServoSetPulseUs(Servo_TypeDef *servo, uint16_t pulse_us);

/* Set 0~angle_max_deg angle. Values outside the range are clamped. */
void ServoSetAngle(Servo_TypeDef *servo, uint16_t angle_deg, uint16_t angle_max_deg);

/* Stop PWM output for this servo channel. */
HAL_StatusTypeDef ServoStop(Servo_TypeDef *servo);

#endif
