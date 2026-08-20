#ifndef __TEST_H_
#define __TEST_H_

#include <stdint.h>

/* 纯虚拟底盘速度规划测试：不访问CAN和电机，只通过USART1输出VOFA JustFloat。 */
void TestChassisSpeedPlanInit(void);
void TestChassisVofaTask(void *argument);

/* 机械臂单项动作调试：返回1时表示已按TEST_ARM_MODE完成初始化并需创建测试任务。 */
uint8_t TestArmMotionInit(void);
void TestArmControlTask(void *argument);
void TestArmMotionTask(void *argument);

#endif /* __TEST_H_ */
