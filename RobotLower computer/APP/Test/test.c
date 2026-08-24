#include "main.h"
#include "test.h"
#include "StrategyAlogrithm.h"
#include "usart.h"
#include "cmsis_os2.h"
#include "Core.h"
#include "GasPump.h"

/* ST-Link调参区：修改a/v/j后重新编译烧录。单位为m/s^2、m/s、m/s^3。 */
#define TEST_CHASSIS_PLAN_A_MAX       (2.0f)
#define TEST_CHASSIS_PLAN_V_MAX       (1.0f)
#define TEST_CHASSIS_PLAN_J           (8.0f)
#define TEST_CHASSIS_PLAN_DISTANCE_M  (1.0f)
#define TEST_CHASSIS_VOFA_PERIOD_MS   (20U)

/*
 * 机械臂调试总开关：正常使用保持 NONE。
 * 单电机定位测试步骤：
 * 1. 确认机械臂周围无障碍物，急停可用；先使用很小的目标位置变化。
 * 2. 将 TEST_ARM_MODE 改为 TEST_ARM_MODE_SINGLE_MOTOR。
 * 3. 在下方选择 TEST_ARM_MOTOR，并填写 TEST_ARM_MOTOR_MOVE_DISTANCE_M。
 * 4. 编译、下载、复位；程序等待 TEST_ARM_START_DELAY_MS 后开始运动。
 * 5. Keil Watch 查看 TestArmMotorReached 和 TestArmFinished：均为 1U 表示到位。
 */
#define TEST_ARM_MODE_NONE             (0U)
#define TEST_ARM_MODE_FLIP             (1U)
#define TEST_ARM_MODE_PICKUP_STORE_LOW (2U)
#define TEST_ARM_MODE_PICKUP_STORE_HIGH (3U)
#define TEST_ARM_MODE_PICKUP_HOLD      (4U)
#define TEST_ARM_MODE_SINGLE_MOTOR     (5U)
#define TEST_ARM_MODE                  TEST_ARM_MODE_SINGLE_MOTOR

/*
 * 单电机位置测试参数。
 * LIFT 和 FORWARD 使用相对移动距离，单位 m；正负号决定运动方向。
 * 任务读取真实反馈位置后，按 RoboticArm.h 的 K 值换算为电机目标角度。
 * 第一次测试建议仅填写 0.005f~0.010f，确认方向后再扩大距离。
 */
#define TEST_ARM_MOTOR_LIFT            (0U) /* J60 升降电机 */
#define TEST_ARM_MOTOR_FORWARD         (1U) /* GO 前后电机 */
#define TEST_ARM_MOTOR                 TEST_ARM_MOTOR_LIFT /* 二选一：LIFT / FORWARD */
#define TEST_ARM_MOTOR_MOVE_DISTANCE_M (0.50f)              /* LIFT/FORWARD 相对移动距离，单位 m */
#define TEST_ARM_MOTOR_TOLERANCE_M     (0.001f)            /* 到位允许误差，单位 m（1 mm） */
/* 当前 J60 升降电机正方向与机械臂“向上”为反向，因此测试中取 -1。 */
#define TEST_ARM_LIFT_DIRECTION        (-1.0f)

/* 拾取测试使用的KFS高度；HOLD测试到位后保持吸附的时长。 */
#define TEST_ARM_PICKUP_HEIGHT         PICKUP_HEIGHT_LOW
#define TEST_ARM_START_DELAY_MS        (3000U)
#define TEST_ARM_HOLD_DURATION_MS      (1000U)

/* JustFloat帧：3个float + 固定帧尾00 00 80 7F。 */
#pragma pack(push, 1)
typedef struct
{
    float data[3];
    uint8_t tail[4];
} TestChassisJustFloatFrame_TypeDef;
#pragma pack(pop)

/* 全局变量可直接在Keil Watch观察。 */
SpeedPlan_TypeDef TestChassisPlan;
float TestChassisVirtualPosition;
float TestChassisVirtualTarget;

/*
 * Keil Watch 调试变量：
 * TestArmActive=1U 表示测试正在运行；0U 表示已结束。
 * TestArmFinished=1U 表示当前测试动作完成。
 * TestArmMotorReached=1U 表示单电机实际位移误差进入 TOLERANCE_M 范围。
 * TestArmMotorActualAngle 是当前测试电机的实际反馈角度，单位 rad。
 * TestArmMotorTargetPosition 是当前测试电机的目标角度，单位 rad。
 */
uint8_t TestArmActive;
uint8_t TestArmFinished;
FlipState_TypeDef TestArmFlipState;
PickupState_TypeDef TestArmPickupState;
uint8_t TestArmMotorReached;
float TestArmMotorActualAngle;
float TestArmMotorTargetPosition;

static TestChassisJustFloatFrame_TypeDef TestChassisVofaFrame =
{
    {0.0f, 0.0f, 0.0f},
    {0x00U, 0x00U, 0x80U, 0x7FU}
};

void TestChassisSpeedPlanInit(void)
{
    SpeedPlanInit(&TestChassisPlan,
                  TEST_CHASSIS_PLAN_A_MAX,
                  TEST_CHASSIS_PLAN_V_MAX,
                  TEST_CHASSIS_PLAN_J);
    TestChassisVirtualPosition = 0.0f;
    TestChassisVirtualTarget = TEST_CHASSIS_PLAN_DISTANCE_M;
}

void TestChassisVofaTask(void *argument)
{
    uint32_t vofa_tick;

    (void)argument;
    vofa_tick = HAL_GetTick();

    for (;;)
    {
        /* 仅推进虚拟规划器，position不来自任何传感器。 */
        SpeedPlanUpdate(&TestChassisPlan,
                        TestChassisVirtualPosition,
                        TestChassisVirtualTarget);
        TestChassisVirtualPosition = TestChassisPlan.position_initial +
                                     TestChassisPlan.direction_flag * TestChassisPlan.s;

        /* 到端点后反向，使VOFA持续显示完整的加减速曲线。 */
        if (TestChassisPlan.state == idle)
        {
            TestChassisVirtualTarget = (TestChassisVirtualTarget > 0.0f) ?
                                      0.0f : TEST_CHASSIS_PLAN_DISTANCE_M;
            TestChassisPlan.state = init;
        }

        if ((HAL_GetTick() - vofa_tick) >= TEST_CHASSIS_VOFA_PERIOD_MS)
        {
            vofa_tick = HAL_GetTick();

            /* VOFA: I0=有符号速度v, I1=有符号加速度a, I2=jerk上限j。 */
            TestChassisVofaFrame.data[0] = TestChassisPlan.v * TestChassisPlan.direction_flag;
            TestChassisVofaFrame.data[1] = TestChassisPlan.a * TestChassisPlan.direction_flag;
            TestChassisVofaFrame.data[2] = TestChassisPlan.j;

            if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
                HAL_UART_Transmit_DMA(&huart1,
                                      (uint8_t *)&TestChassisVofaFrame,
                                      sizeof(TestChassisVofaFrame));
        }

        osDelay(1);
    }
}

uint8_t TestArmMotionInit(void)
{
#if TEST_ARM_MODE == TEST_ARM_MODE_NONE
    return 0U;
#else
    /* 回调分发使用全局Robot，因此调试也复用正式机械臂初始化路径。 */
    RobotInit();
    TestArmActive = 1U;
    TestArmFinished = 0U;
    TestArmFlipState = FLIP_STATE_UP;
    TestArmPickupState = PICKUP_STATE_RAISE;
    TestArmMotorReached = 0U;
    TestArmMotorActualAngle = 0.0f;
    TestArmMotorTargetPosition = 0.0f;
    return 1U;
#endif
}

void TestArmControlTask(void *argument)
{
    (void)argument;

    /* 持续下发三轴目标、请求GO反馈并更新末端实际坐标。 */
    RoboticArmUpdate(&Robot.roboticarm);
}

void TestArmMotionTask(void *argument)
{
#if TEST_ARM_MODE == TEST_ARM_MODE_PICKUP_HOLD
    uint32_t hold_start_tick = 0U;
#endif

    (void)argument;
    osDelay(TEST_ARM_START_DELAY_MS);

#if TEST_ARM_MODE == TEST_ARM_MODE_SINGLE_MOTOR
    /* 延时结束后只下发一次目标；控制任务持续发送该目标并刷新反馈。 */
#if TEST_ARM_MOTOR == TEST_ARM_MOTOR_LIFT
    /* delta_height = LIFT_K * delta_theta，因此 delta_theta = delta_height / LIFT_K。 */
    TestArmMotorTargetPosition = Robot.roboticarm.lift_motor.feedback.position +
                                 TEST_ARM_LIFT_DIRECTION * TEST_ARM_MOTOR_MOVE_DISTANCE_M /
                                 ROBOTICARM_LIFT_K;
    J60MotorSetTarget(&Robot.roboticarm.lift_motor, TestArmMotorTargetPosition);
#elif TEST_ARM_MOTOR == TEST_ARM_MOTOR_FORWARD
    /* delta_distance = FORWARD_K * delta_theta，因此 delta_theta = delta_distance / FORWARD_K。 */
    TestArmMotorTargetPosition = Robot.roboticarm.go_motors.motors[ROBOTICARM_GO_FORWARD].feedback.position +
                                 TEST_ARM_MOTOR_MOVE_DISTANCE_M / ROBOTICARM_FORWARD_K;
    GOM8010GroupSetTarget(&Robot.roboticarm.go_motors, ROBOTICARM_GO_FORWARD,
                          TestArmMotorTargetPosition);
#else
#error "TEST_ARM_MOTOR must be LIFT or FORWARD"
#endif
#endif

    while (TestArmActive != 0U)
    {
#if TEST_ARM_MODE == TEST_ARM_MODE_FLIP
        RoboticArmFlipMotion(&Robot.roboticarm, &TestArmFlipState);
        if (TestArmFlipState == FLIP_STATE_DONE)
            TestArmFinished = 1U;
#elif TEST_ARM_MODE == TEST_ARM_MODE_PICKUP_STORE_LOW
        RoboticArmPickupStoreLowMotion(&Robot.roboticarm, &TestArmPickupState,
                                       TEST_ARM_PICKUP_HEIGHT);
        if (TestArmPickupState == PICKUP_STATE_VOID)
            TestArmFinished = 1U;
#elif TEST_ARM_MODE == TEST_ARM_MODE_PICKUP_STORE_HIGH
        RoboticArmPickupStoreHighMotion(&Robot.roboticarm, &TestArmPickupState,
                                        TEST_ARM_PICKUP_HEIGHT);
        if (TestArmPickupState == PICKUP_STATE_VOID)
            TestArmFinished = 1U;
#elif TEST_ARM_MODE == TEST_ARM_MODE_PICKUP_HOLD
        RoboticArmPickupHoldMotion(&Robot.roboticarm, &TestArmPickupState,
                                   TEST_ARM_PICKUP_HEIGHT);
        if (TestArmPickupState == PICKUP_STATE_HOLD)
        {
            if (hold_start_tick == 0U)
                hold_start_tick = HAL_GetTick();
            else if ((HAL_GetTick() - hold_start_tick) >= TEST_ARM_HOLD_DURATION_MS)
                TestArmFinished = 1U;
        }
#elif TEST_ARM_MODE == TEST_ARM_MODE_SINGLE_MOTOR
        /* 将电机角度误差按机构 K 值换算为米，再判断是否到位。 */
#if TEST_ARM_MOTOR == TEST_ARM_MOTOR_LIFT
        float position_error_m = (Robot.roboticarm.lift_motor.feedback.position -
                                  TestArmMotorTargetPosition) * ROBOTICARM_LIFT_K;
        TestArmMotorActualAngle = Robot.roboticarm.lift_motor.feedback.position;
#elif TEST_ARM_MOTOR == TEST_ARM_MOTOR_FORWARD
        float position_error_m = (Robot.roboticarm.go_motors.motors[ROBOTICARM_GO_FORWARD].feedback.position -
                                  TestArmMotorTargetPosition) * ROBOTICARM_FORWARD_K;
        TestArmMotorActualAngle = Robot.roboticarm.go_motors.motors[ROBOTICARM_GO_FORWARD].feedback.position;
#else
#error "TEST_ARM_MOTOR must be LIFT or FORWARD"
#endif
        if (position_error_m < 0.0f)
            position_error_m = -position_error_m;
        if (position_error_m <= TEST_ARM_MOTOR_TOLERANCE_M)
        {
            TestArmMotorReached = 1U;
            TestArmFinished = 1U;
        }
#else
        TestArmFinished = 1U;
#endif

        if (TestArmFinished != 0U)
        {
            /* 单电机测试不操作气泵；所有电机保持最后一个已到位目标。 */
#if TEST_ARM_MODE != TEST_ARM_MODE_SINGLE_MOTOR
            RoboticArmReleaseMotion();
#endif
            TestArmActive = 0U;
        }

        osDelay(ROBOT_STATE_UPDATE_PERIOD_MS);
    }

    for (;;)
    {
        osDelay(osWaitForever);
    }
}
