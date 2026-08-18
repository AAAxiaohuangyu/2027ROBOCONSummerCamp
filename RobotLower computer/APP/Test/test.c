#include "test.h"
#include "Core.h"
#include "bsp_config.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <stdio.h>

/* 测试下发的前进线速度幅值,单位与ChassisSetVelocity一致(m/s) */
#define TEST_CHASSIS_LINEAR_VELOCITY_MPS (0.3f)

/* 上电后延迟这么久再下发前进指令,预留时间摆放小车/接好huart1串口调试线 */
#define TEST_CHASSIS_STARTUP_DELAY_MS (2000U)

/* 通过huart1广播即将下发的vx/vy/wz,格式:"<NAME> vx=.. vy=.. wz=..\n",便于核对实际
   轮子转动方向/转速是否符合预期;测试期间不会调用Vision_Init,借用huart1不会与视觉模块冲突 */
static void TestChassisReportStep(const char *name, float vx_mps, float vy_mps, float wz_radps)
{
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "%s vx=%.3f vy=%.3f wz=%.3f\n",
                       name, (double)vx_mps, (double)vy_mps, (double)wz_radps);
    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, HAL_MAX_DELAY);
    }
}

/* 只拉起Robot.chassis相关外设(对齐Core.c::RobotInit中底盘部分),不初始化机械臂/Zigbee/视觉/
   翻转/拾取,不进入RobotStateUpdate状态机,便于单独测试ChassisSetVelocity */
void TestChassisSetVelocityInit(void)
{
    ChassisInit(&Robot.chassis, CHASSIS_FDCAN_HANDLE, CHASSIS_CTRL_ID);

    if (Robot.chassis.drive.motor_group.FDCAN_Handle != NULL)
    {
        FDCANStandardInit(Robot.chassis.drive.motor_group.FDCAN_Handle,
                          M3508_FEEDBACK_ID_BASE + M3508_ID_MIN,
                          M3508_FEEDBACK_ID_BASE + M3508_ID_MAX);
    }
}

/* 持续推进底盘控制环(麦轮逆解+电机指令下发);内部为死循环,需作为独立任务体运行 */
void TestChassisUpdateTask(void *argument)
{
    (void)argument;
    ChassisUpdate(&Robot.chassis);
}

/* 上电延迟TEST_CHASSIS_STARTUP_DELAY_MS后,通过huart1报告并下发一次前进速度,此后不再下发
   停止指令,小车持续前进;ChassisSetVelocity只是设定目标,由TestChassisUpdateTask持续维持,
   故此任务下发一次即可结束。调用前须已执行过TestChassisSetVelocityInit */
void TestChassisSetVelocityTask(void *argument)
{

    osDelay(TEST_CHASSIS_STARTUP_DELAY_MS);

    while(1)
    {
                ChassisSetVelocity(&Robot.chassis, TEST_CHASSIS_LINEAR_VELOCITY_MPS, 0.0f, 0.0f);
    }
}
