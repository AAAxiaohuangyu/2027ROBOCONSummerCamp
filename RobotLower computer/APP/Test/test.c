#include "test.h"
#include "Core.h"
#include "bsp_config.h"
#include "cmsis_os.h"

static EncoderTask_TypeDef TestEncoderTask;

void TestChassisEncoderRxInit(void)
{
    ChassisInit(&Robot.chassis, CHASSIS_FDCAN_HANDLE, CHASSIS_CTRL_ID);
    FDCANStandardInit(Robot.chassis.drive.motor_group.FDCAN_Handle,
                       M3508_FEEDBACK_ID_BASE + M3508_ID_MIN,
                       M3508_FEEDBACK_ID_BASE + M3508_ID_MAX); /* 过滤器0 -> RXFIFO0 */

    EncoderInit(&Robot.encoder, &TestEncoderTask,
                ENCODER_X_FDCAN_HANDLE, ENCODER_X_NODE_ID,
                ENCODER_Y_FDCAN_HANDLE, ENCODER_Y_NODE_ID,
                NULL, NULL); /* 过滤器0 -> RXFIFO1 */
}

void TestChassisUpdateTask(void *argument)
{
    (void)argument;
    ChassisUpdate(&Robot.chassis); /* ChassisInit后velocity=0/mode=1,持续下发0电流控制帧触发电调反馈 */
}

void TestEncoderUpdateTask(void *argument)
{
    (void)argument;
    EncoderUpdate(&TestEncoderTask); /* 周期发位置请求触发encoder应答 */
}

void TestChassisSetVelocityTask(void *argument)
{
    for (;;)
    {
        ChassisSetVelocity(&Robot.chassis, 0.0f, 0.0f, 0.0f); /* 固定测试速度,验证电机能否按目标转动;TestChassisUpdateTask持续下发该速度对应的控制帧 */
        osDelay(500);
    }
}
