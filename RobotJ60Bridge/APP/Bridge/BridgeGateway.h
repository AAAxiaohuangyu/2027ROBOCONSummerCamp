#ifndef __BRIDGE_GATEWAY_H__
#define __BRIDGE_GATEWAY_H__

#include "fdcan.h"
#include "usart.h"

#define BRIDGE_SOF0       (0xA5U)
#define BRIDGE_SOF1       (0x5AU)
#define BRIDGE_TYPE_CAN_TX (0x01U)
#define BRIDGE_TYPE_CAN_RX (0x81U)
#define BRIDGE_FRAME_SIZE (15U)

void BridgeGateway_Init(UART_HandleTypeDef *huart, FDCAN_HandleTypeDef *hfdcan);
void BridgeGateway_UartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void BridgeGateway_CanRxEvent(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_its);

#endif
