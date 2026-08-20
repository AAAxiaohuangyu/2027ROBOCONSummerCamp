#ifndef __ENCODER_BAUDRATE_SWITCH_H_
#define __ENCODER_BAUDRATE_SWITCH_H_

#include <stdint.h>

/* 单只BRT38M码盘CAN波特率切换测试。 */
uint8_t EncoderBaudrateSwitchInit(void);
void EncoderBaudrateSwitchTask(void *argument);

extern uint8_t EncoderBaudrateSwitchStatus;

#endif /* __ENCODER_BAUDRATE_SWITCH_H_ */
