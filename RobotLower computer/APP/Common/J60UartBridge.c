#include "J60UartBridge.h"
#include <string.h>

static uint8_t J60BridgeCrc8(const uint8_t *data, uint16_t length)
{
    /* 对 TYPE 至 DATA 的所有字节做异或校验，帧头不参与，便于接收端重新同步。 */
    uint8_t crc = 0U;
    uint16_t i;

    for (i = 0U; i < length; ++i)
        crc ^= data[i];

    return crc;
}

HAL_StatusTypeDef J60UartBridge_Init(J60UartBridge_TypeDef *bridge, UART_HandleTypeDef *huart)
{
    if (bridge == NULL || huart == NULL)
        return HAL_ERROR;

    /* 清空统计值和 DMA 接收缓冲后，启动一帧定长的空闲 DMA 接收。 */
    memset(bridge, 0, sizeof(*bridge));
    bridge->huart = huart;

    if (HAL_UARTEx_ReceiveToIdle_DMA(huart, bridge->rx_dma_buf, J60_BRIDGE_FRAME_SIZE) != HAL_OK)
        return HAL_ERROR;

    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    return HAL_OK;
}

HAL_StatusTypeDef J60UartBridge_SendCanFrame(J60UartBridge_TypeDef *bridge, uint16_t can_id,
                                             const uint8_t data[J60_CAN_FRAME_LENGTH], uint8_t dlc)
{
    uint8_t frame[J60_BRIDGE_FRAME_SIZE] = {0};

    if (bridge == NULL || bridge->huart == NULL || dlc > J60_CAN_FRAME_LENGTH)
        return HAL_ERROR;

    /* 串口始终发送 15 字节：未使用的 DATA 字节保留为 0，DLC 指示有效长度。 */
    frame[0] = J60_BRIDGE_SOF0;
    frame[1] = J60_BRIDGE_SOF1;
    frame[2] = J60_BRIDGE_TYPE_CAN_TX;
    frame[3] = (uint8_t)can_id;
    frame[4] = (uint8_t)(can_id >> 8U);
    frame[5] = dlc;
    if (dlc != 0U)
        memcpy(&frame[6], data, dlc);
    frame[J60_BRIDGE_FRAME_SIZE - 1U] = J60BridgeCrc8(&frame[2], J60_BRIDGE_FRAME_SIZE - 3U);

    /* J60 控制周期为 3 ms，2 ms 超时用于及时发现新增板或串口链路异常。 */
    if (HAL_UART_Transmit(bridge->huart, frame, J60_BRIDGE_FRAME_SIZE, 2U) != HAL_OK)
        return HAL_ERROR;

    bridge->tx_count++;
    return HAL_OK;
}

void J60UartBridge_RxEventHandler(J60UartBridge_TypeDef *bridge, UART_HandleTypeDef *huart,
                                  uint16_t size, J60Motor_TypeDef *motor)
{
    const uint8_t *frame = bridge->rx_dma_buf;
    uint16_t can_id;

    /* 仅接受新增板回传的完整 8 字节 J60 反馈 CAN 帧，避免把命令或异常数据写入反馈。 */
    if (size == J60_BRIDGE_FRAME_SIZE && frame[0] == J60_BRIDGE_SOF0 &&
        frame[1] == J60_BRIDGE_SOF1 && frame[2] == J60_BRIDGE_TYPE_CAN_RX &&
        frame[5] <= J60_CAN_FRAME_LENGTH &&
        frame[J60_BRIDGE_FRAME_SIZE - 1U] == J60BridgeCrc8(&frame[2], J60_BRIDGE_FRAME_SIZE - 3U))
    {
        can_id = (uint16_t)frame[3] | ((uint16_t)frame[4] << 8U);
        /* 复用原 J60 CAN 反馈解析，因此上层位置控制无需感知 bridge 的存在。 */
        if (frame[5] == J60_CAN_FRAME_LENGTH && J60MotorParseFeedback(motor, can_id, &frame[6]))
            bridge->rx_count++;
        else
            bridge->error_count++;
    }
    else
    {
        bridge->error_count++;
    }

    /* DMA 普通模式在每次空闲事件后停止，必须重新挂起下一帧接收。 */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, bridge->rx_dma_buf, J60_BRIDGE_FRAME_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
