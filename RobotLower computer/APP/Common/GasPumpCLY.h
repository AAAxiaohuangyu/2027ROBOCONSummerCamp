#ifndef __GASPUMP_H__
#define __GASPUMP_H__

#include "stm32h7xx_hal.h"

/* GPIO配置宏定义 */
#define GASPUMP_GPIO_PORT GPIOB   
#define GASPUMP_GPIO_PIN GPIO_PIN_3

#define CLY_GPIO_PORT GPIOD
#define CLY_GPIO_PIN GPIO_PIN_7
/* 状态定义 */
#define GASPUMP_ON GPIO_PIN_SET    /* 气泵开启状态 */
#define GASPUMP_OFF GPIO_PIN_RESET /* 气泵关闭状态 */

#define CLY_ON GPIO_PIN_RESET //吸盘吸住
#define CLY_OFF GPIO_PIN_SET //吸盘释放
/* 函数声明 */
void GasPumpOn(void);
void GasPumpOff(void);
void CLY_On(void);
void CLY_Off(void);

#endif
