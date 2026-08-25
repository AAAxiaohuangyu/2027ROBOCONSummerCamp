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

/* ------------------------- 本项目舵机标定与硬件参数 ------------------------- */
/* 舵机上电时的初始脉宽，当前机械标定为 1000 us。 */
#define SERVO_INITIAL_PULSE_US       1000U
/* 从初始位置转到 180 度时增加的脉宽，当前标定增量为 1390 us。 */
#define SERVO_MOVE_PULSE_US          1390U
/* 一次相对运动（0->180 或 180->0）的等待时间，单位 ms。 */
#define SERVO_MOVE_TIME_MS           1390U
/* 上电输出初始脉宽后的稳定等待时间，单位 ms。 */
#define SERVO_TEST_SETTLE_TIME_MS    1000U
/* 舵机到达 180 度后的保持时间，期间电磁阀断电，单位 ms。 */
#define SERVO_VALVE_TIME_MS          1000U

/* 新板 JP6 舵机接口：PB5 信号、GND、5 V；PB5 复用为 TIM3_CH2。 */
#define SERVO_PWM_TIMER              TIM3
#define SERVO_PWM_CHANNEL            TIM_CHANNEL_2
#define SERVO_PWM_GPIO_PORT          GPIOB
#define SERVO_PWM_GPIO_PIN           GPIO_PIN_5
#define SERVO_PWM_GPIO_AF            GPIO_AF2_TIM3
/* 定时器计数频率为 1 MHz，因此 1 个计数对应 1 us。 */
#define SERVO_PWM_TIMER_TICK_HZ      1000000U
/* PWM 周期为 20 ms，即标准舵机控制频率 50 Hz。 */
#define SERVO_PWM_PERIOD_US          20000U

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t timer_tick_hz; /* 定时器计数频率；1 MHz 时 1 个计数等于 1 us。 */
    uint16_t pulse_min_us;
    uint16_t pulse_max_us;
    uint16_t pulse_us;
} Servo_TypeDef;

/*
 * 绑定已配置的 PWM 定时器通道并启动输出。定时器 ARR 对应的周期必须大于
 * pulse_max_us；本项目使用 20 ms 周期和 1 MHz 计数频率。
 */
HAL_StatusTypeDef ServoInit(Servo_TypeDef *servo, TIM_HandleTypeDef *htim,
                            uint32_t channel, uint32_t timer_tick_hz,
                            uint16_t pulse_min_us, uint16_t pulse_max_us,
                            uint16_t initial_pulse_us);

/* 设置高电平脉宽（us）；超出最小/最大脉宽的值会自动限幅。 */
void ServoSetPulseUs(Servo_TypeDef *servo, uint16_t pulse_us);

/* 设置 0~angle_max_deg 角度，并按当前标定区间线性换算为脉宽。 */
void ServoSetAngle(Servo_TypeDef *servo, uint16_t angle_deg, uint16_t angle_max_deg);

/* 停止该舵机通道的 PWM 输出。 */
HAL_StatusTypeDef ServoStop(Servo_TypeDef *servo);

#endif
