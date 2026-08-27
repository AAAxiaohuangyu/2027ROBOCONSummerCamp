#include "Servo.h"
#include <string.h>
#include "cmsis_os2.h"

void ServoInit(Servo_TypeDef *servo, TIM_HandleTypeDef *htim,uint32_t channel)
{
    if ((servo == NULL) || (htim == NULL))
        return;

    memset(servo, 0, sizeof(*servo));
    servo->htim = htim;
    servo->channel = channel;
    __HAL_TIM_SET_COMPARE(htim, channel, SERVO_INITIAL_PULSE_US);
    HAL_TIM_PWM_Start(htim, channel);
    return;
}

void ServoAngleUpdate(Servo_TypeDef *servo)
{
        if (servo == NULL)
            return;

        if (servo->angle > SERVO_MAX_ANGLE)
        {
            servo->angle = SERVO_MAX_ANGLE;
        }
        else if (servo->angle < SERVO_MIN_ANGLE)
        {
            servo->angle = SERVO_MIN_ANGLE;
        }

        uint32_t compare = (uint32_t)(SERVO_INITIAL_PULSE_US + servo->angle * SERVO_TRANS_PARAMETER);

        __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, compare);

        osDelay(SERVO_CONTROL_PERIOD);
}
