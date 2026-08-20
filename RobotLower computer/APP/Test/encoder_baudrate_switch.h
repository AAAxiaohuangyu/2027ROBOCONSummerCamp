#ifndef __ENCODER_BAUDRATE_SWITCH_H_
#define __ENCODER_BAUDRATE_SWITCH_H_

#include <stdint.h>

/*
 * 单只 BRT38M 码盘 CAN 波特率切换测试接口。
 * 这两个函数由 freertos.c 调用；使用者通常只需修改 .c 文件顶部的 MODE 和 NODE_ID。
 */
uint8_t EncoderBaudrateSwitchInit(void);
void EncoderBaudrateSwitchTask(void *argument);

/* 在 Keil Watch 中查看本变量，状态值解释见 encoder_baudrate_switch.c。 */
extern uint8_t EncoderBaudrateSwitchStatus;

#endif /* __ENCODER_BAUDRATE_SWITCH_H_ */
