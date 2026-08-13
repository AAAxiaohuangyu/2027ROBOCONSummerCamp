#ifndef __GASPUMP_H__
#define __GASPUMP_H__

#include "stm32h7xx_hal.h"

/* GPIO配置宏定义 */
#define GASPUMP_GPIO_PORT GPIOA     /* 气泵控制GPIO端口 */
#define GASPUMP_GPIO_PIN GPIO_PIN_0 /* 气泵控制GPIO引脚 */

/* 气泵状态定义 */
#define GASPUMP_ON GPIO_PIN_SET    /* 气泵开启状态 */
#define GASPUMP_OFF GPIO_PIN_RESET /* 气泵关闭状态 */

/* 函数声明 */
void RoboticArmGripMotion(void);
void RoboticArmReleaseMotion(void);

#endif
