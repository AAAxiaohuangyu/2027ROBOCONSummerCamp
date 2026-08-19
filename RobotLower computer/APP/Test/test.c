#include "test.h"
#include "Core.h"
#include "bsp_config.h"
#include "cmsis_os.h"

void TestChassisEncoderRxInit(void)
{
    HAL_Delay(1000);
    ChassisInit(&Robot.chassis, CHASSIS_FDCAN_HANDLE, CHASSIS_CTRL_ID);
    FDCANStandardInit(Robot.chassis.drive.motor_group.FDCAN_Handle,
                       M3508_FEEDBACK_ID_BASE + M3508_ID_MIN,
                       M3508_FEEDBACK_ID_BASE + M3508_ID_MAX); /* 过滤器0 -> RXFIFO0 */

    EncoderInit(&Robot.encoder,
                ENCODER_X_FDCAN_HANDLE, ENCODER_X_NODE_ID,
                ENCODER_Y_FDCAN_HANDLE, ENCODER_Y_NODE_ID); /* 过滤器0 -> RXFIFO1 */
}

void TestChassisUpdateTask(void *argument)
{
    (void)argument;
    ChassisUpdate(&Robot.chassis); /* ChassisInit后velocity=0/mode=1,持续下发0电流控制帧触发电调反馈 */
}

void TestEncoderUpdateTask(void *argument)
{
    (void)argument;
    EncoderUpdate(&Robot.encoder); /* 周期发位置请求触发encoder应答 */
}

void TestChassisSetVelocityTask(void *argument)
{
    for (;;)
    {
        ChassisSetVelocity(&Robot.chassis, 0.0f, 0.0f, 0.0f); /* 固定测试速度,验证电机能否按目标转动;TestChassisUpdateTask持续下发该速度对应的控制帧 */
        osDelay(100);
    }
}

/* TestChassisSetPositionTask状态机各状态:START_*只在进入时下发一次ChassisSetTranslation
   (该函数每次调用都会令S曲线重新规划,跑向同一目标期间不能重复调用),WAIT_*则只轮询
   ChassisTranslationReached等待到位;到位后对两只编码器做一次setmid重置并等待10ms,
   再沿y方向平移,到位后保持不动 */
typedef enum
{
    TEST_POSITION_STATE_START_MOVE_X,
    TEST_POSITION_STATE_WAIT_MOVE_X,
    TEST_POSITION_STATE_RESET_MID,
    TEST_POSITION_STATE_START_MOVE_Y,
    TEST_POSITION_STATE_WAIT_MOVE_Y,
    TEST_POSITION_STATE_DONE,
} TestPositionState_TypeDef;

void TestChassisSetPositionTask(void *argument)
{
    TestPositionState_TypeDef state = TEST_POSITION_STATE_START_MOVE_X;

    (void)argument;

    for (;;)
    {
        switch (state)
        {
        case TEST_POSITION_STATE_START_MOVE_X:
            ChassisSetTranslation(&Robot.chassis, -2.0f, 0.0f); /* x方向,只在进入本状态时下发一次 */
            state = TEST_POSITION_STATE_WAIT_MOVE_X;
            break;

        case TEST_POSITION_STATE_WAIT_MOVE_X:
            if (ChassisTranslationReached(&Robot.chassis, ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                state = TEST_POSITION_STATE_RESET_MID;
            break;

        case TEST_POSITION_STATE_RESET_MID:
            EncoderRequestSetMidpoint(&Robot.encoder);
            osDelay(10);
            state = TEST_POSITION_STATE_START_MOVE_Y;
            break;

        case TEST_POSITION_STATE_START_MOVE_Y:
            ChassisSetTranslation(&Robot.chassis, -2.0f, 1.0f); /* y方向,只在进入本状态时下发一次 */
            state = TEST_POSITION_STATE_WAIT_MOVE_Y;
            break;

        case TEST_POSITION_STATE_WAIT_MOVE_Y:
            if (ChassisTranslationReached(&Robot.chassis, ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                state = TEST_POSITION_STATE_DONE;
            break;

        case TEST_POSITION_STATE_DONE:
        default:
            break;
        }

        osDelay(ROBOT_STATE_UPDATE_PERIOD_MS);
    }
}
