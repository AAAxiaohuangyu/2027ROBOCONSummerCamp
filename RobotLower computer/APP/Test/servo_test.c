#include "servo_test.h"
#include "Servo.h"
#include "GasPump.h"

#define SERVO_TEST_INITIAL_PULSE_US  SERVO_INITIAL_PULSE_US
#define SERVO_TEST_MOVE_PULSE_US     SERVO_MOVE_PULSE_US
#define SERVO_TEST_MOVE_TIME_MS      SERVO_MOVE_TIME_MS

static TIM_HandleTypeDef htim3_servo;
static Servo_TypeDef servo;
static uint32_t action_tick;
static uint8_t action_state;

enum
{
    SERVO_TEST_ROTATE = 0U,
    SERVO_TEST_WAIT_RELEASE,
    SERVO_TEST_WAIT_HOME,
    SERVO_TEST_DONE,
};

HAL_StatusTypeDef ServoTestInit(void)
{
    /*
     * 此处不依赖 CubeMX 自动生成的 TIM3 初始化函数，而是在测试模块中完成
     * PB5/TIM3_CH2 的最小配置。这样烧录本测试固件后，只有舵机 PWM 被启用。
     */
    TIM_OC_InitTypeDef oc = {0};
    GPIO_InitTypeDef gpio = {0};
    uint32_t timer_clock_hz;

    /* 配置 JP6 信号脚：PB5 的 AF2 输出连接到 TIM3_CH2。 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    gpio.Pin = SERVO_PWM_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = SERVO_PWM_GPIO_AF;
    HAL_GPIO_Init(SERVO_PWM_GPIO_PORT, &gpio);

    /*
     * 当前工程 APB1 为 HCLK/2，通用定时器内核时钟为 PCLK1 的两倍。
     * 例如当前 TIM3 时钟为 64 MHz，预分频值设为 63 后得到 1 MHz，
     * 因而比较寄存器 CCR 的数值可直接使用微秒单位。
     */
    timer_clock_hz = HAL_RCC_GetPCLK1Freq() * 2U;
    htim3_servo.Instance = SERVO_PWM_TIMER;
    htim3_servo.Init.Prescaler = (timer_clock_hz / SERVO_PWM_TIMER_TICK_HZ) - 1U;
    htim3_servo.Init.CounterMode = TIM_COUNTERMODE_UP;
    /* ARR=19999，PWM 周期为 20000 us，即标准舵机需要的 50 Hz。 */
    htim3_servo.Init.Period = SERVO_PWM_PERIOD_US - 1U;
    htim3_servo.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3_servo) != HAL_OK)
        return HAL_ERROR;

    /* PWM1 模式下计数器小于 CCR 时输出高电平，1500 us 通常为舵机中位。 */
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 1500U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3_servo, &oc, SERVO_PWM_CHANNEL) != HAL_OK)
        return HAL_ERROR;

    /* 上电首先输出已标定的启动脉宽，固定舵机的实际机械初始位置。 */
    if (ServoInit(&servo, &htim3_servo, SERVO_PWM_CHANNEL,
                  SERVO_PWM_TIMER_TICK_HZ, SERVO_DEFAULT_MIN_PULSE_US,
                  SERVO_DEFAULT_MAX_PULSE_US, SERVO_TEST_INITIAL_PULSE_US) != HAL_OK)
        return HAL_ERROR;

    /*
     * 软件不能读取舵机实际轴角，SERVO_TEST_INITIAL_PULSE_US 是本次测试的
     * 人工标定初始位置。ServoTestUpdate() 等待稳定后再在该脉宽基础上增加
     * SERVO_TEST_MOVE_PULSE_US，即本次标定的相对 +180 度运动。
     */
    /* CLY1(PD7) 作为电磁阀：旋转阶段断电；气泵本身由外部保持持续吸气。 */
    RoboticArmReleaseMotion();
    ServoSetPulseUs(&servo, SERVO_TEST_INITIAL_PULSE_US);
    action_tick = HAL_GetTick();
    action_state = SERVO_TEST_ROTATE;
    return HAL_OK;
}

void ServoTestUpdate(void)
{
    uint32_t target_pulse_us;

    switch (action_state)
    {
    case SERVO_TEST_ROTATE:
        if ((HAL_GetTick() - action_tick) < SERVO_TEST_SETTLE_TIME_MS)
            return;
        target_pulse_us = SERVO_TEST_INITIAL_PULSE_US + SERVO_TEST_MOVE_PULSE_US;
        ServoSetPulseUs(&servo, (uint16_t)target_pulse_us);
        action_tick = HAL_GetTick();
        action_state = SERVO_TEST_WAIT_RELEASE;
        break;

    case SERVO_TEST_WAIT_RELEASE:
        /* 先给舵机足够时间完成 0->180 度运动。 */
        if ((HAL_GetTick() - action_tick) < SERVO_MOVE_TIME_MS)
            return;
        /* 到达 180 度后，再额外保持 1 秒，期间 CLY1 仍断电。 */
        if ((HAL_GetTick() - action_tick) <
            (SERVO_MOVE_TIME_MS + SERVO_VALVE_TIME_MS))
            return;
        action_tick = HAL_GetTick();
        action_state = SERVO_TEST_WAIT_HOME;
        ServoSetPulseUs(&servo, SERVO_TEST_INITIAL_PULSE_US);
        break;

    case SERVO_TEST_WAIT_HOME:
        /* 等待舵机返回 0 度；返回过程中 CLY1 继续保持断电。 */
        if ((HAL_GetTick() - action_tick) < SERVO_MOVE_TIME_MS)
            return;
        /* 舵机返回 0 度后，CLY1(PD7) 通电，电磁阀打开并释放 KFS。 */
        RoboticArmGripMotion();
        action_state = SERVO_TEST_DONE;
        break;

    case SERVO_TEST_DONE:
    default:
        /* 舵机已返回初始脉宽；保持 PWM，电磁阀保持通电状态。 */
        break;
    }
}
