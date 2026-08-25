#include "test.h"
#include "Core.h"
#include "bsp_config.h"
#include "chassis_config.h"
#include "ControlAlgorithm.h"
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

    /* 开启UART9空闲线DMA接收;之后每帧到达都由HAL_UARTEx_RxEventCallback(bsp_callback.c)
       中断驱动解析,持续更新Robot.zigbee.explained_data,不需要额外的周期任务 */
    Zigbee_Init(&Robot.zigbee);

    Vision_Init(&Robot.vision);
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

/* TestChassisSetPositionTask状态机各状态:START_*只在进入时下发一次ChassisSetTranslation
   (该函数每次调用都会令S曲线重新规划,跑向同一目标期间不能重复调用;x/y现为世界系绝对
   目标,START_MOVE_Y必须原样带上START_MOVE_X已下发的x目标,否则x会被打断拉回0);
   WAIT_MOVE_Y用固定容差轮询ChassisTranslationReached等待终点到位,WAIT_MOVE_X则用
   TEST_CORNER_BLEND_K放大的动态容差提前判定"到位",在x轴尚未停稳前就切到
   START_MOVE_Y以实现转弯圆滑过渡,而非在拐角处完全停顿 */
typedef enum
{
    TEST_POSITION_STATE_START_MOVE_1,
    TEST_POSITION_STATE_WAIT_MOVE_1,
    TEST_POSITION_STATE_START_MOVE_2,
    TEST_POSITION_STATE_WAIT_MOVE_2,
    TEST_POSITION_STATE_START_MOVE_3,
    TEST_POSITION_STATE_WAIT_MOVE_3,
    TEST_POSITION_STATE_START_MOVE_4,
    TEST_POSITION_STATE_WAIT_MOVE_4,
    TEST_POSITION_STATE_START_MOVE_5,
    TEST_POSITION_STATE_WAIT_MOVE_5,
    TEST_POSITION_STATE_START_MOVE_6,
    TEST_POSITION_STATE_WAIT_MOVE_6,
    TEST_POSITION_STATE_START_MOVE_7,
    TEST_POSITION_STATE_WAIT_MOVE_7,
    TEST_POSITION_STATE_START_MOVE_8,
    TEST_POSITION_STATE_WAIT_MOVE_8,
    TEST_POSITION_STATE_START_MOVE_9,
    TEST_POSITION_STATE_WAIT_MOVE_9,
    TEST_MANUAL_STATE,
    TEST_POSITION_STATE_DONE,
} TestPositionState_TypeDef;

/* 转弯提前系数(无量纲):WAIT_MOVE_X用ChassisTranslationReached判断是否
   切换到START_MOVE_Y时,传入的容差为该系数乘以x轴当前速度对应的S曲线
   减速距离(SpeedPlanDecelDistance),而非固定的米数,因此能随速度规划
   参数自动适配;k=1表示x刚进入减速阶段就切换,转弯越圆滑需要越大的k,
   但也会让实际路径更早偏离(-2,0)这个中间点 */
#define TEST_CORNER_BLEND_K (1.7f)

/* MOVE_9(上斜坡)：码盘 y 轴世界系累计位置(Robot.encoder.y_m,与 chassis->pose.y_m
   同一坐标系)到达该值附近即认为冲坡到位,暂定,需实测标定后手动修改 */
#define TEST_MOVE_9_ENCODER_Y_TARGET_M (-10.38f)

/* MOVE_5~MOVE_8阶段x跟踪固定切到第三套参数组,MOVE_6~MOVE_8阶段y轴固定切到备用
   速度规划/跟踪参数组;这两次切换本应分别发生在START_MOVE_5/START_MOVE_6状态,
   若该阶段被KFS_DIFF跳过则由TestSkipMove补做一次,以保证后续阶段仍使用正确参数 */
static void TestApplyMove5Params(void)
{
    PIDInit(&Robot.chassis.displacement_plan.translation_x.track_pid,
            CHASSIS_TRACK_TRANSLATION_X_ALT2_KP,
            CHASSIS_TRACK_TRANSLATION_X_ALT2_KI,
            CHASSIS_TRACK_TRANSLATION_X_ALT2_KD,
            CHASSIS_TRACK_TRANSLATION_X_ALT2_MAX_OUT,
            CHASSIS_TRACK_TRANSLATION_X_ALT2_MAX_IOUT);
}

static void TestApplyMove6Params(void)
{
    SpeedPlanInit(&Robot.chassis.displacement_plan.translation_y,
                  CHASSIS_PLAN_TRANSLATION_Y_ALT_MAX_ACCEL_MPS2,
                  CHASSIS_PLAN_TRANSLATION_Y_ALT_MAX_SPEED_MPS,
                  CHASSIS_PLAN_TRANSLATION_Y_ALT_MAX_JERK_MPS3,
                  CHASSIS_TRACK_TRANSLATION_DEADBAND_M);
    PIDInit(&Robot.chassis.displacement_plan.translation_y.track_pid,
            CHASSIS_TRACK_TRANSLATION_Y_ALT_KP,
            CHASSIS_TRACK_TRANSLATION_Y_ALT_KI,
            CHASSIS_TRACK_TRANSLATION_Y_ALT_KD,
            CHASSIS_TRACK_TRANSLATION_Y_ALT_MAX_OUT,
            CHASSIS_TRACK_TRANSLATION_Y_ALT_MAX_IOUT);
}

/* MOVE_9(上斜坡)阶段x跟踪切到第四套参数组(不重新调用ChassisSetTranslation,
   仅切换跟踪增益) */
static void TestApplyMove9Params(void)
{
    PIDInit(&Robot.chassis.displacement_plan.translation_x.track_pid,
            CHASSIS_TRACK_TRANSLATION_X_ALT3_KP,
            CHASSIS_TRACK_TRANSLATION_X_ALT3_KI,
            CHASSIS_TRACK_TRANSLATION_X_ALT3_KD,
            CHASSIS_TRACK_TRANSLATION_X_ALT3_MAX_OUT,
            CHASSIS_TRACK_TRANSLATION_X_ALT3_MAX_IOUT);
}

/* Robot.vision.KFS_DIFF为1~5时分别跳过阶段5~9(即KFS_DIFF+4那一段的START/WAIT状态,
   直接进入下一阶段;跳过阶段9则直接进入TEST_MANUAL_STATE),其余KFS_DIFF值不处理 */
static TestPositionState_TypeDef TestSkipMove(uint8_t move_num, TestPositionState_TypeDef next_state)
{
    if (Robot.vision.KFS_DIFF > 0 && Robot.vision.KFS_DIFF < 5 && move_num == (uint8_t)(Robot.vision.KFS_DIFF + 4))
    {
        if (move_num == 5)
            TestApplyMove5Params();
        else if (move_num == 6)
            TestApplyMove6Params();

        return (move_num == 9) ? TEST_MANUAL_STATE : (TestPositionState_TypeDef)(next_state + 2);
    }
    return next_state;
}

void TestChassisSetPositionTask(void *argument)
{
    TestPositionState_TypeDef state = TEST_POSITION_STATE_START_MOVE_9;
    Robot.vision.KFS_DIFF = 0;

    (void)argument;

    for (;;)
    {

        if (Robot.zigbee.rx_data.command.mode == 1)
        {
            state = TEST_MANUAL_STATE;
        }

        switch (state)
        {
        case TEST_POSITION_STATE_START_MOVE_1:
            osDelay(500);
            ChassisSetTranslation(&Robot.chassis, 0.8f, -0.65f);
            state = TEST_POSITION_STATE_WAIT_MOVE_1;
            break;

        case TEST_POSITION_STATE_WAIT_MOVE_1:
        {
            float corner_tolerance = TEST_CORNER_BLEND_K *
                                     SpeedPlanDecelDistance(&Robot.chassis.displacement_plan.translation_x);
            if (corner_tolerance < ROBOT_CHASSIS_POSITION_TOLERANCE_M)
                corner_tolerance = ROBOT_CHASSIS_POSITION_TOLERANCE_M;

            if (ChassisTranslationReached(&Robot.chassis, corner_tolerance))
                state = TEST_POSITION_STATE_START_MOVE_2;
            break;
        }

        case TEST_POSITION_STATE_START_MOVE_2:
            ChassisSetTranslation(&Robot.chassis, 3.3f, -0.65f);
            state = TEST_POSITION_STATE_WAIT_MOVE_2;
            break;

        case TEST_POSITION_STATE_WAIT_MOVE_2:
        {
            float corner_tolerance = (TEST_CORNER_BLEND_K + 0.3f) *
                                     SpeedPlanDecelDistance(&Robot.chassis.displacement_plan.translation_x);
            if (corner_tolerance < ROBOT_CHASSIS_POSITION_TOLERANCE_M)
                corner_tolerance = ROBOT_CHASSIS_POSITION_TOLERANCE_M;
            if (ChassisTranslationReached(&Robot.chassis, corner_tolerance))
                state = TEST_POSITION_STATE_START_MOVE_3;
            break;
        }

        case TEST_POSITION_STATE_START_MOVE_3:
        {
            ChassisSetTranslation(&Robot.chassis, 3.3f, -0.1f);
            state = TEST_POSITION_STATE_WAIT_MOVE_3;
            break;
        }

        case TEST_POSITION_STATE_WAIT_MOVE_3:
        {
            if (ChassisTranslationReached(&Robot.chassis, 4.0f * ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                state = TEST_POSITION_STATE_START_MOVE_4;
            break;
        }

        case TEST_POSITION_STATE_START_MOVE_4:
        {
            osDelay(800);
            /* MOVE_4/WAIT_MOVE_4阶段x方向回退幅度大,临时切到备用跟踪参数组,
               离开该阶段(START_MOVE_5)后切回默认组 */
            PIDInit(&Robot.chassis.displacement_plan.translation_x.track_pid,
                    CHASSIS_TRACK_TRANSLATION_X_ALT_KP,
                    CHASSIS_TRACK_TRANSLATION_X_ALT_KI,
                    CHASSIS_TRACK_TRANSLATION_X_ALT_KD,
                    CHASSIS_TRACK_TRANSLATION_X_ALT_MAX_OUT,
                    CHASSIS_TRACK_TRANSLATION_X_ALT_MAX_IOUT);
            ChassisSetTranslation(&Robot.chassis, 0.4f, -2.1f);
            state = TEST_POSITION_STATE_WAIT_MOVE_4;
            break;
        }

        case TEST_POSITION_STATE_WAIT_MOVE_4:
        {
            float corner_tolerance = (TEST_CORNER_BLEND_K)*SpeedPlanDecelDistance(&Robot.chassis.displacement_plan.translation_x);
            if (corner_tolerance < ROBOT_CHASSIS_POSITION_TOLERANCE_M)
                corner_tolerance = ROBOT_CHASSIS_POSITION_TOLERANCE_M;
            if (ChassisTranslationReached(&Robot.chassis, corner_tolerance))
                state = TestSkipMove(5, TEST_POSITION_STATE_START_MOVE_5);
            break;
        }

        case TEST_POSITION_STATE_START_MOVE_5:
        {
            /* 离开MOVE_4/WAIT_MOVE_4,MOVE_5~MOVE_8阶段x跟踪切到第三套参数组;
               y轴的备用速度规划/跟踪参数组从MOVE_6起才切换 */
            TestApplyMove5Params();
            ChassisSetTranslation(&Robot.chassis, 0.4f, -3.42f);
            state = TEST_POSITION_STATE_WAIT_MOVE_5;
            break;
        }

        case TEST_POSITION_STATE_WAIT_MOVE_5:
        {
            if (ChassisTranslationReached(&Robot.chassis, 3.0f * ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                state = TestSkipMove(6, TEST_POSITION_STATE_START_MOVE_6);
            break;
        }

        case TEST_POSITION_STATE_START_MOVE_6:
        {
            osDelay(1000);
            /* MOVE_6~MOVE_8阶段y轴切到备用S曲线速度规划参数组(x轴规划器不受影响)、
               y跟踪也切到备用参数组 */
            TestApplyMove6Params();
            ChassisSetTranslation(&Robot.chassis, 0.4f, -4.6f);
            state = TEST_POSITION_STATE_WAIT_MOVE_6;
            break;
        }

        case TEST_POSITION_STATE_WAIT_MOVE_6:
        {
            if (ChassisTranslationReached(&Robot.chassis, 2.0f * ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                state = TestSkipMove(7, TEST_POSITION_STATE_START_MOVE_7);
            break;
        }

        case TEST_POSITION_STATE_START_MOVE_7:
        {
            osDelay(1000);
            ChassisSetTranslation(&Robot.chassis, 0.4f, -5.79f);
            state = TEST_POSITION_STATE_WAIT_MOVE_7;
            break;
        }

        case TEST_POSITION_STATE_WAIT_MOVE_7:
        {
            if (ChassisTranslationReached(&Robot.chassis, 2.0f * ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                state = TestSkipMove(8, TEST_POSITION_STATE_START_MOVE_8);
            break;
        }

        case TEST_POSITION_STATE_START_MOVE_8:
        {
            osDelay(1000);
            ChassisSetTranslation(&Robot.chassis, 0.4f, -6.98f);
            state = TEST_POSITION_STATE_WAIT_MOVE_8;
            break;
        }

        case TEST_POSITION_STATE_WAIT_MOVE_8:
        {
            if (ChassisTranslationReached(&Robot.chassis, 2.0f * ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                state = TestSkipMove(9, TEST_POSITION_STATE_START_MOVE_9);
            break;
        }

        case TEST_POSITION_STATE_START_MOVE_9:
        {
            /* x、偏航沿用 MOVE_8 已下发的目标继续跟踪(不重新调用
               ChassisSetTranslation/ChassisSetYaw),x跟踪切到第四套参数组,
               仅 y 轴切到固定速度冲坡 */
            osDelay(1000);
            TestApplyMove9Params();
            ChassisSetRampVelocity(&Robot.chassis, -1.2f);
            state = TEST_POSITION_STATE_WAIT_MOVE_9;
            break;
        }

        case TEST_POSITION_STATE_WAIT_MOVE_9:
        {
            if (Robot.encoder.y_m <= TEST_MOVE_9_ENCODER_Y_TARGET_M)
            {
                state = TEST_MANUAL_STATE;
            }
            break;
        }

        case TEST_MANUAL_STATE:
        {
            ChassisSetVelocity(&Robot.chassis, Robot.zigbee.rx_data.chassis.speed_vx, Robot.zigbee.rx_data.chassis.speed_vy, Robot.zigbee.rx_data.chassis.omega);

            if (Robot.zigbee.rx_data.command.emergency_stop == 1)
            {
                ChassisStop(&Robot.chassis);
            }

            break;
        }

        case TEST_POSITION_STATE_DONE:
        default:
            break;
        }

        osDelay(ROBOT_STATE_UPDATE_PERIOD_MS);
    }
}
