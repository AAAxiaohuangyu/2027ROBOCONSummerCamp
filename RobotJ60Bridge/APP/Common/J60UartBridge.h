#ifndef __J60_UART_BRIDGE_H__
#define __J60_UART_BRIDGE_H__

#include "J60.h"
#include "usart.h"

/*
 * 主控 USART1 <-> 新增板网关帧（固定 15 字节）：
 * SOF(2) + type(1) + CAN ID(2，小端) + DLC(1) + CAN data(8) + CRC8(1)。
 * TYPE_CAN_TX 由主控发送，新增板原样转发至 FDCAN2；TYPE_CAN_RX 由新增板回传 J60 反馈。
 */
#define J60_BRIDGE_SOF0             (0xA5U)
#define J60_BRIDGE_SOF1             (0x5AU)
#define J60_BRIDGE_TYPE_CAN_TX      (0x01U)
#define J60_BRIDGE_TYPE_CAN_RX      (0x81U)
#define J60_BRIDGE_FRAME_SIZE       (15U)

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t rx_dma_buf[J60_BRIDGE_FRAME_SIZE];
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
} J60UartBridge_TypeDef;

HAL_StatusTypeDef J60UartBridge_Init(J60UartBridge_TypeDef *bridge, UART_HandleTypeDef *huart);
HAL_StatusTypeDef J60UartBridge_SendCanFrame(J60UartBridge_TypeDef *bridge, uint16_t can_id,
                                             const uint8_t data[J60_CAN_FRAME_LENGTH], uint8_t dlc);
void J60UartBridge_RxEventHandler(J60UartBridge_TypeDef *bridge, UART_HandleTypeDef *huart,
                                  uint16_t size, J60Motor_TypeDef *motor);

#endif
