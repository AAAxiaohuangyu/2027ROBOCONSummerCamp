#include "RoboticArm.h"
#include "cmsis_os2.h"

static TIM_HandleTypeDef htim3_rotate_servo;

/* 配置 JP6 的 PB5/TIM3_CH2。1500 us 为舵机上电保持的机械初始位置。 */
static HAL_StatusTypeDef RoboticArmRotateServoInit(Servo_TypeDef *servo)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};
    uint32_t timer_clock_hz;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    gpio.Pin = SERVO_PWM_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = SERVO_PWM_GPIO_AF;
    HAL_GPIO_Init(SERVO_PWM_GPIO_PORT, &gpio);

    timer_clock_hz = HAL_RCC_GetPCLK1Freq() * 2U;
    htim3_rotate_servo.Instance = SERVO_PWM_TIMER;
    htim3_rotate_servo.Init.Prescaler = (timer_clock_hz / SERVO_PWM_TIMER_TICK_HZ) - 1U;
    htim3_rotate_servo.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3_rotate_servo.Init.Period = SERVO_PWM_PERIOD_US - 1U;
    htim3_rotate_servo.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3_rotate_servo.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim3_rotate_servo) != HAL_OK)
        return HAL_ERROR;

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = SERVO_INITIAL_PULSE_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3_rotate_servo, &oc, SERVO_PWM_CHANNEL) != HAL_OK)
        return HAL_ERROR;

    return ServoInit(servo, &htim3_rotate_servo, SERVO_PWM_CHANNEL,
                     SERVO_PWM_TIMER_TICK_HZ, SERVO_DEFAULT_MIN_PULSE_US,
                     SERVO_DEFAULT_MAX_PULSE_US, SERVO_INITIAL_PULSE_US);
}

/* 根据三电机当前反馈转角,按几何关系正解出末端坐标与杆自转角度,写回arm对应字段 */
static void
RoboticArmUpdateStateFromFeedback(RoboticArm_TypeDef *arm)
{
    /*
     * 机械臂正解：根据三个轴已经解析完成的输出端反馈，计算末端当前状态。
     * 减速比由 J60/GO 驱动处理，本函数只使用经过实物标定的机械比例和零点偏置。
     */
    const GOM8010Feedback_TypeDef *forward_feedback = &arm->go_motors.motors[ROBOTICARM_GO_FORWARD].feedback;
    float height = ROBOTICARM_LIFT_K * arm->lift_motor.feedback.position + ROBOTICARM_LIFT_THRESHOLD;
    float distance = ROBOTICARM_FORWARD_K * forward_feedback->position + ROBOTICARM_FORWARD_THRESHOLD;

    arm->end_x = ROBOTICARM_BASE_X + distance;
    arm->end_y = ROBOTICARM_BASE_Y + ROBOTICARM_ROD_LENGTH;
    arm->end_z = height + ROBOTICARM_END_Z_OFFSET;
}

void RoboticArmInit(RoboticArm_TypeDef *arm,
                    FDCAN_HandleTypeDef *lift_FDCAN_Handle, uint8_t lift_id,
                    UART_HandleTypeDef *forward_huart, uint8_t forward_id)
{
    /*
     * 此处只建立软件对象：J60 还需显式使能，GO 的实际控制帧需由后续周期任务发送。
     * 两台 GO 必须按固定下标加入组：0 为前后轴，1 为自转轴，供上层统一访问。
     */
    J60MotorInit(&arm->lift_motor, lift_FDCAN_Handle, lift_id);

    GOM8010GroupInit(&arm->go_motors);
    /* GO 组仅保留 RS485-2/UART4 上的前后轴 ID 3；自转轴改由 PWM 舵机控制。 */
    GOM8010GroupAddMotor(&arm->go_motors, forward_id, forward_huart);
    if (RoboticArmRotateServoInit(&arm->rotate_servo) != HAL_OK)
    {
        Error_Handler();
    }
    arm->rod_rotation = 0.0f;
    arm->rotate_command_tick = HAL_GetTick();
    arm->rotate_motion_active = 0U;

    RoboticArmUpdateStateFromFeedback(arm);
}

void RoboticArmSetEndPosition(RoboticArm_TypeDef *arm, float end_x_target, float end_z_target,
                              float lift_torque_feedforward)
{
    /*
     * 机械臂反解：调用者输入 m 单位的末端目标，函数将其转为两个输出端 rad 目标。
     * SetTarget 会重新开始 S 曲线；上层进入一个动作时应下发一次，等待阶段只检查到位，
     * 不要每个周期重复调用，否则规划器会持续重新起步。
     */
    float distance_target = end_x_target - ROBOTICARM_BASE_X;
    float height_target = end_z_target - ROBOTICARM_END_Z_OFFSET;

    float theta_forward_target = (distance_target - ROBOTICARM_FORWARD_THRESHOLD) / ROBOTICARM_FORWARD_K;
    float theta_lift_target = (height_target - ROBOTICARM_LIFT_THRESHOLD) / ROBOTICARM_LIFT_K;

    J60MotorSetTarget(&arm->lift_motor, theta_lift_target);
    J60MotorSetTorqueFeedforward(&arm->lift_motor, lift_torque_feedforward);
    GOM8010GroupSetTarget(&arm->go_motors, ROBOTICARM_GO_FORWARD, theta_forward_target);
}

void RoboticArmSetRodRotation(RoboticArm_TypeDef *arm, float rotation_target)
{
    uint16_t servo_pulse_us;
    float target_error;

    /* ID 7 替换为 180 度舵机：0 rad 对应 0 度，pi rad 对应 180 度。 */
    if (rotation_target < 0.0f)
        rotation_target = 0.0f;
    if (rotation_target > BSP_PI)
        rotation_target = BSP_PI;

    /* Flip/Pickup 会按周期重复提出同一目标；相同目标不能重复启动运动计时。 */
    target_error = rotation_target - arm->rod_rotation;
    if (target_error < 0.0f)
        target_error = -target_error;
    if ((target_error < 0.0001f) && (arm->rotate_motion_active != 0U))
        return;

    servo_pulse_us = (uint16_t)((float)SERVO_INITIAL_PULSE_US +
                                (float)SERVO_MOVE_PULSE_US * rotation_target / BSP_PI);
    ServoSetPulseUs(&arm->rotate_servo, servo_pulse_us);
    arm->rotate_command_tick = HAL_GetTick();
    arm->rotate_motion_active = 1U;
    arm->rod_rotation = rotation_target;
}

void RoboticArmUpdate(RoboticArm_TypeDef *arm)
{
    /*
     * 这是长期运行的执行任务入口，内部已有无限循环和 osDelay，应作为独立任务创建。
     * 它负责发送控制帧并更新状态；不能嵌套调用进另一个同样长期运行的状态机任务。
     */
    while (1)
    {
        J60MotorUpdate(&arm->lift_motor);
        GOM8010GroupUpdate(&arm->go_motors);

        RoboticArmUpdateStateFromFeedback(arm);
        osDelay(ROBOTICARM_CONTROL_PERIOD_MS);
    }
}

void RoboticArmEnable(RoboticArm_TypeDef *arm)
{
    J60MotorEnable(&arm->lift_motor);
}

void RoboticArmDisable(RoboticArm_TypeDef *arm)
{
    J60MotorDisable(&arm->lift_motor);
}

uint8_t PositionReached(RoboticArm_TypeDef *arm,
                        float target_x,
                        float target_z, float x_position_tolerance, float z_position_tolrance)
{
    /* 以末端反馈坐标判断到位，而不是直接比较电机角度；两个方向可使用不同容差。 */
    /* 使用末端反馈坐标判断 x/z 目标是否到位。 */
    float dx = arm->end_x - target_x;
    float dz = arm->end_z - target_z;

    if (dx < 0.0f)
    {
        dx = -dx;
    }

    if (dz < 0.0f)
    {
        dz = -dz;
    }

    return (dx <= x_position_tolerance &&
            dz <= z_position_tolrance);
}

uint8_t RotationReached(RoboticArm_TypeDef *arm,
                        float target_rotation, float rotation_tolerance)
{
    float error;

    /* 普通 PWM 舵机无位置反馈，必须等待标定的运动时间后才能认为到位。 */
    if (arm->rotate_motion_active != 0U)
    {
        if ((HAL_GetTick() - arm->rotate_command_tick) < SERVO_MOVE_TIME_MS)
            return 0U;
        arm->rotate_motion_active = 0U;
    }

    error = arm->rod_rotation - target_rotation;

    if (error < 0.0f)
    {
        error = -error;
    }

    return (error <= rotation_tolerance);
}
