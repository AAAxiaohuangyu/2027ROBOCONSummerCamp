#ifndef __GASPUMP_H__
#define __GASPUMP_H__

#include "stm32h7xx_hal.h"

/*
 * 气泵模块只暴露“抓取/释放”两个语义接口。上层动作模块不应直接调用
 * HAL_GPIO_WritePin，避免各模块各自假定相反的有效电平。GPIO 引脚与 ON/OFF
 * 电平必须依据当前板卡原理图和 CubeMX 输出配置确认。
 */

/* GPIO配置宏定义 */
#define GASPUMP_GPIO_PORT GPIOD     /* 长门板 CLY1_IN 所在端口；对应板载 ULN2001D 的第一路输出 */
#define GASPUMP_GPIO_PIN GPIO_PIN_7 /* 长门板 CLY1_IN；默认假设吸盘的控制线接在 CLY1_OUT */

/* 气泵状态定义 */
#define GASPUMP_ON GPIO_PIN_SET    /* 气泵开启状态 */
#define GASPUMP_OFF GPIO_PIN_RESET /* 气泵关闭状态 */

/* 函数声明 */
void RoboticArmGripMotion(void);
void RoboticArmReleaseMotion(void);

#endif
