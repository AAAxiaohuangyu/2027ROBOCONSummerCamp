#include "ServoDebug.h"
#include "Servo.h"

#define SERVO_DEBUG_TIMER_TICK_HZ 1000000U
#define SERVO_DEBUG_PERIOD_US     20000U
#define SERVO_DEBUG_STEP_MS       2000U

static TIM_HandleTypeDef htim3_servo;
static Servo_TypeDef servo_debug;
static uint32_t servo_debug_last_tick;
static uint8_t servo_debug_step;

HAL_StatusTypeDef ServoDebugInit(void)
{
    TIM_OC_InitTypeDef oc = {0};
    GPIO_InitTypeDef gpio = {0};
    uint32_t timer_clock_hz;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* JP6 pin 3 is PB5/TIM3_CH2 alternate function 2 on STM32H723. */
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* APB1 is HCLK/2; TIM2 clock is doubled to 64 MHz when APB prescaler is not 1. */
    timer_clock_hz = HAL_RCC_GetPCLK1Freq() * 2U;
    htim3_servo.Instance = TIM3;
    htim3_servo.Init.Prescaler = (timer_clock_hz / SERVO_DEBUG_TIMER_TICK_HZ) - 1U;
    htim3_servo.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3_servo.Init.Period = SERVO_DEBUG_PERIOD_US - 1U;
    htim3_servo.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3_servo) != HAL_OK)
        return HAL_ERROR;

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 1500U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3_servo, &oc, TIM_CHANNEL_2) != HAL_OK)
        return HAL_ERROR;

    if (ServoInit(&servo_debug, &htim3_servo, TIM_CHANNEL_2,
                  SERVO_DEBUG_TIMER_TICK_HZ, SERVO_DEFAULT_MIN_PULSE_US,
                  SERVO_DEFAULT_MAX_PULSE_US, 1500U) != HAL_OK)
        return HAL_ERROR;

    servo_debug_last_tick = HAL_GetTick();
    servo_debug_step = 0U;
    return HAL_OK;
}

void ServoDebugUpdate(void)
{
    static const uint16_t angles[] = {0U, 90U, 180U};

    if ((HAL_GetTick() - servo_debug_last_tick) < SERVO_DEBUG_STEP_MS)
        return;

    servo_debug_last_tick = HAL_GetTick();
    ServoSetAngle(&servo_debug, angles[servo_debug_step], SERVO_DEFAULT_ANGLE_MAX);
    servo_debug_step = (servo_debug_step + 1U) % (sizeof(angles) / sizeof(angles[0]));
}
