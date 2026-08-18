#ifndef __TEST_H_
#define __TEST_H_

#include "main.h"

/* ChassisSetVelocity(原始速度接口)独立联调测试:不经过Core.c的整机状态机(RobotInit/
   RobotStateUpdate),只单独拉起Robot.chassis,不初始化机械臂/Zigbee/视觉/翻转/拾取等其余
   板块。上电后延迟一段时间,自动下发一次前进vx(仅前进,不再下发停止指令,持续前进),不依赖
   上位机交互,通过huart1广播下发的速度值,便于对照实际轮子转动方向/转速是否符合预期。测试
   期间不会调用Vision_Init,借用huart1不会与视觉模块冲突。

   使用方式:在freertos.c的MX_FREERTOS_Init中,先(在创建线程之前)同步调用一次
   TestChassisSetVelocityInit完成底盘外设初始化,再分别为TestChassisUpdateTask、
   TestChassisSetVelocityTask各创建一个线程(前者需持续运行,推进底盘控制环并下发电机指令;
   后者下发一次前进速度后即退出)。Init必须在两个线程开始运行前完成,否则两个线程都会访问
   尚未初始化的Robot.chassis。 */

void TestChassisSetVelocityInit(void);
void TestChassisUpdateTask(void *argument);
void TestChassisSetVelocityTask(void *argument);

#endif
