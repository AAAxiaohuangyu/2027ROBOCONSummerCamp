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

    Yis512Init(&Robot.yis512); /* 扩展过滤器0 -> RXFIFO0 */
}

void TestChassisUpdateTask(void *argument)
{
    (void)argument;
    ChassisUpdate(&Robot.chassis, &Robot.yis512); /* ChassisInit后velocity=0/mode=1,持续下发0电流控制帧触发电调反馈 */
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
   (该函数每次调用都会令S曲线重新规划,跑向同一目标期间不能重复调用;x/y现为世界系绝对
   目标,START_MOVE_Y必须原样带上START_MOVE_X已下发的x目标,否则x会被打断拉回0);
   WAIT_MOVE_Y用固定容差轮询ChassisTranslationReached等待终点到位,WAIT_MOVE_X则用
   TEST_CORNER_BLEND_K放大的动态容差提前判定"到位",在x轴尚未停稳前就切到
   START_MOVE_Y以实现转弯圆滑过渡,而非在拐角处完全停顿 */
typedef enum
{
    TEST_POSITION_STATE_START_MOVE_X,
    TEST_POSITION_STATE_WAIT_MOVE_X,
    TEST_POSITION_STATE_START_MOVE_Y,
    TEST_POSITION_STATE_WAIT_MOVE_Y,
    TEST_POSITION_STATE_DONE,
} TestPositionState_TypeDef;

/* 转弯提前系数(无量纲):WAIT_MOVE_X用ChassisTranslationReached判断是否
   切换到START_MOVE_Y时,传入的容差为该系数乘以x轴当前速度对应的S曲线
   减速距离(SpeedPlanDecelDistance),而非固定的米数,因此能随速度规划
   参数自动适配;k=1表示x刚进入减速阶段就切换,转弯越圆滑需要越大的k,
   但也会让实际路径更早偏离(-2,0)这个中间点 */
#define TEST_CORNER_BLEND_K (1.7f)

void TestChassisSetPositionTask(void *argument)
{
    TestPositionState_TypeDef state = TEST_POSITION_STATE_START_MOVE_X;

    (void)argument;

    for (;;)
    {
        switch (state)
        {
        case TEST_POSITION_STATE_START_MOVE_X:
            osDelay(500);
            ChassisSetTranslation(&Robot.chassis, 2.0f, 0.0f); /* 绝对目标(-2, 0),只在进入本状态时下发一次 */
            state = TEST_POSITION_STATE_DONE;
            break;

        case TEST_POSITION_STATE_WAIT_MOVE_X:
        {
            float corner_tolerance = TEST_CORNER_BLEND_K *
                SpeedPlanDecelDistance(&Robot.chassis.displacement_plan.translation_x);
            if (corner_tolerance < ROBOT_CHASSIS_POSITION_TOLERANCE_M)
                corner_tolerance = ROBOT_CHASSIS_POSITION_TOLERANCE_M;

            if (ChassisTranslationReached(&Robot.chassis, corner_tolerance))
                state = TEST_POSITION_STATE_START_MOVE_Y;
            break;
        }

        case TEST_POSITION_STATE_START_MOVE_Y:
            ChassisSetTranslation(&Robot.chassis, 0.0f, 2.0f); /* 绝对目标(-2, 2),x原样带上避免被打断拉回0 */
            state = TEST_POSITION_STATE_DONE;
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
