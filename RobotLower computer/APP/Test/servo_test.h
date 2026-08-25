#ifndef __SERVO_TEST_H__
#define __SERVO_TEST_H__

#include "stm32h7xx_hal.h"

/*
 * 单舵机上电测试模块。
 *
 * 新板 JP6 从上到下依次为：PB5 信号、GND、5 V。标准三线舵机可直接插接；
 * 若舵机改用外部 5 V 电源，外部电源 GND 与开发板 GND 必须连接在一起。
 * PB5 配置为 AF2 后输出 TIM3_CH2 的 PWM。初始化结束后，驱动保持输出 50 Hz
 * PWM。普通 PWM 舵机没有位置回读能力，启动机械位置通过 servo_test.c 中的
 * SERVO_TEST_INITIAL_PULSE_US 脉宽标定；上电先保持该位置，稳定后只执行一次
 * 相对 +90 度运动。
 */
HAL_StatusTypeDef ServoTestInit(void);
void ServoTestUpdate(void);

#endif /* __SERVO_TEST_H__ */
