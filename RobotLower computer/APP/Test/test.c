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

/* 机械臂单项调试选择：默认关闭。修改TEST_ARM_MODE、重新编译烧录后生效。 */
#define TEST_ARM_MODE_NONE             (0U)
#define TEST_ARM_MODE_FLIP             (1U)
#define TEST_ARM_MODE_PICKUP_STORE_LOW (2U)
#define TEST_ARM_MODE_PICKUP_STORE_HIGH (3U)
#define TEST_ARM_MODE_PICKUP_HOLD      (4U)
#define TEST_ARM_MODE                  TEST_ARM_MODE_NONE

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

/* 全局变量可直接在Keil Watch观察测试模式和状态机进度。 */
uint8_t TestArmActive;
uint8_t TestArmFinished;
FlipState_TypeDef TestArmFlipState;
PickupState_TypeDef TestArmPickupState;

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
#else
        TestArmFinished = 1U;
#endif

        if (TestArmFinished != 0U)
        {
            /* 所有测试结束时关闭气泵；电机保持最后一个已到位目标。 */
            RoboticArmReleaseMotion();
            TestArmActive = 0U;
        }

        osDelay(ROBOT_STATE_UPDATE_PERIOD_MS);
    }

    for (;;)
    {
        osDelay(osWaitForever);
    }
}
