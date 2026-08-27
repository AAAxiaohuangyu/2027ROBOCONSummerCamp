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

    /* memset 后 servo->angle 为 0 rad，按反向映射应输出 2390 us。 */
    __HAL_TIM_SET_COMPARE(htim, channel, SERVO_ZERO_ANGLE_PULSE_US);
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

        /*
         * 反向角度映射：
         *   0 rad  -> 2390 us；
         *   PI rad -> 1000 us。
         * 角度增大时 PWM 脉宽减小，使舵机的实际转动方向与原映射相反。
         */
        uint32_t compare = (uint32_t)(SERVO_ZERO_ANGLE_PULSE_US -
                                      servo->angle * SERVO_TRANS_PARAMETER);

        __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, compare);

        osDelay(SERVO_CONTROL_PERIOD);
}
