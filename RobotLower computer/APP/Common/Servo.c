#include "Servo.h"
#include <string.h>

static uint16_t ServoClampPulse(const Servo_TypeDef *servo, uint16_t pulse_us)
{
    if (pulse_us < servo->pulse_min_us)
        return servo->pulse_min_us;
    if (pulse_us > servo->pulse_max_us)
        return servo->pulse_max_us;
    return pulse_us;
}

static uint32_t ServoPulseToCompare(const Servo_TypeDef *servo, uint16_t pulse_us)
{
    return ((uint32_t)pulse_us * servo->timer_tick_hz + 500000U) / 1000000U;
}

HAL_StatusTypeDef ServoInit(Servo_TypeDef *servo, TIM_HandleTypeDef *htim,
                            uint32_t channel, uint32_t timer_tick_hz,
                            uint16_t pulse_min_us, uint16_t pulse_max_us,
                            uint16_t initial_pulse_us)
{
    if ((servo == NULL) || (htim == NULL) || (timer_tick_hz == 0U) ||
        (pulse_min_us >= pulse_max_us))
        return HAL_ERROR;

    memset(servo, 0, sizeof(*servo));
    servo->htim = htim;
    servo->channel = channel;
    servo->timer_tick_hz = timer_tick_hz;
    servo->pulse_min_us = pulse_min_us;
    servo->pulse_max_us = pulse_max_us;

    ServoSetPulseUs(servo, initial_pulse_us);
    return HAL_TIM_PWM_Start(htim, channel);
}

void ServoSetPulseUs(Servo_TypeDef *servo, uint16_t pulse_us)
{
    if ((servo == NULL) || (servo->htim == NULL) || (servo->timer_tick_hz == 0U))
        return;

    servo->pulse_us = ServoClampPulse(servo, pulse_us);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel,
                          ServoPulseToCompare(servo, servo->pulse_us));
}

void ServoSetAngle(Servo_TypeDef *servo, uint16_t angle_deg, uint16_t angle_max_deg)
{
    uint32_t pulse_us;

    if ((servo == NULL) || (angle_max_deg == 0U))
        return;

    if (angle_deg > angle_max_deg)
        angle_deg = angle_max_deg;

    pulse_us = servo->pulse_min_us +
               (((uint32_t)(servo->pulse_max_us - servo->pulse_min_us) * angle_deg) +
                (angle_max_deg / 2U)) / angle_max_deg;
    ServoSetPulseUs(servo, (uint16_t)pulse_us);
}

HAL_StatusTypeDef ServoStop(Servo_TypeDef *servo)
{
    if ((servo == NULL) || (servo->htim == NULL))
        return HAL_ERROR;

    return HAL_TIM_PWM_Stop(servo->htim, servo->channel);
}
