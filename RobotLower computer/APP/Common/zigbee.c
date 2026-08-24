/**
 * @file    zigbee.c
 * @brief   ZigBee 透传模块驱动实现
 *
 * 帧格式：
 * | SOF0 | SOF1 | LEN | PAYLOAD |
 *
 * 调用说明：
 *   1. 在 MX_USART1_UART_Init() 之后调用 Zigbee_Init(&Robot.zigbee)
 *   2. 在周期任务（≤100ms）中调用 Zigbee_ErrorHandler(&Robot.zigbee)
 *   3. Zigbee_Send(&Robot.zigbee, &control_data)的调用大约在100-200Hz，确保数据完整性和实时性
 *   4. 接收时采用中断接收，定时解析
 */
#include "zigbee.h"
#include "usart.h"
#include <string.h>
#include "bsp_config.h"

/* 将 ZigbeeData_TypeDef 按大端字节序拼入 buf */
static void data_pack(uint8_t *buf, const ZigbeeData_TypeDef *data)
{
    uint8_t command_byte = 0U;
    memcpy(&buf[0], &data->chassis.speed_vx, sizeof(float));
    memcpy(&buf[4], &data->chassis.speed_vy, sizeof(float));
    memcpy(&buf[8], &data->chassis.omega, sizeof(float));
    memcpy(&buf[12], &data->joint.front_back, sizeof(float));
    memcpy(&buf[16], &data->joint.up_down, sizeof(float));
    memcpy(&buf[20], &data->joint.flip, sizeof(float));
    command_byte |= (uint8_t)((data->command.grab & 0x01U) << 0U);
    command_byte |= (uint8_t)((data->command.emergency_stop & 0x01U) << 1U);
    command_byte |= (uint8_t)((data->command.mode & 0x01U) << 2U);
    buf[24] = command_byte;
}

/* 从 buf按大端字节序解析到 ZigbeeData_TypeDef */
static void data_unpack(ZigbeeData_TypeDef *data, const uint8_t *buf)
{
    memcpy(&data->chassis.speed_vx, &buf[0], sizeof(float));
    memcpy(&data->chassis.speed_vy, &buf[4], sizeof(float));
    memcpy(&data->chassis.omega, &buf[8], sizeof(float));
    memcpy(&data->joint.front_back, &buf[12], sizeof(float));
    memcpy(&data->joint.up_down, &buf[16], sizeof(float));
    memcpy(&data->joint.flip, &buf[20], sizeof(float));
    data->command.grab = (buf[24] >> 0U) & 0x01U;
    data->command.emergency_stop = (buf[24] >> 1U) & 0x01U;
    data->command.mode = (buf[24] >> 2U) & 0x01U;
}

/* 从 DMA 缓冲中搜索并解析一帧 */
static void frame_parse(ZigbeeHandle_TypeDef *zigbee, const uint8_t *buf, uint16_t len)
{
    const uint16_t min_len = (uint16_t)ZIGBEE_FRAME_OVERHEAD + ZIGBEE_PAYLOAD_LEN;
    if (len < min_len) return;

    for (uint16_t i = 0U; (uint16_t)(i + min_len) <= len; i++)
    {
        if (buf[i] != ZIGBEE_FRAME_SOF0 || buf[i + 1U] != ZIGBEE_FRAME_SOF1)
            continue;

        uint8_t  payload_len = buf[i + 2U];
        uint16_t frame_total = (uint16_t)ZIGBEE_FRAME_OVERHEAD + payload_len;

        if ((uint16_t)(i + frame_total) > len) break;  /* 数据不完整 */

        if (payload_len != ZIGBEE_PAYLOAD_LEN)
        {
            zigbee->status.error_count++;
            continue;
        }

        data_unpack(&zigbee->rx_data, &buf[i + 3U]);
        zigbee->rx_valid = 1U;
        zigbee->status.rx_count++;
        zigbee->status.last_rx_tick = HAL_GetTick();
        zigbee->status.state = ZIGBEE_STATE_CONNECTED;
        
        return;
    }
}

//用于屏幕显示
const ZigbeeStatus_TypeDef *Zigbee_GetStatus(const ZigbeeHandle_TypeDef *zigbee)
{
    return &zigbee->status;
}

const uint8_t *Zigbee_GetATResponse(const ZigbeeHandle_TypeDef *zigbee)
{
    return zigbee->at_response;
}

HAL_StatusTypeDef Zigbee_Init(ZigbeeHandle_TypeDef *zigbee)
{
    memset(&zigbee->status, 0, sizeof(zigbee->status));
    memset(&zigbee->rx_data, 0, sizeof(zigbee->rx_data));
    zigbee->rx_valid = 0U;
    zigbee->status.state = ZIGBEE_STATE_DISCONNECTED;
    zigbee->status.last_rx_tick = HAL_GetTick(); /* 防止上电立即误判超时 */

    HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(&ZIGBEE_UART_HANDLE, zigbee->rx_dma_buf, ZIGBEE_RX_BUF_SIZE);

    if (ret != HAL_OK)
        zigbee->status.state = ZIGBEE_STATE_ERROR;

    return ret;
}

HAL_StatusTypeDef Zigbee_Send(ZigbeeHandle_TypeDef *zigbee, const ZigbeeData_TypeDef *data)
{
    if (data == NULL) return HAL_ERROR;

    /* 保存最近一次待发送的数据，便于调试观察及状态追踪。 */
    zigbee->tx_data = *data;

    zigbee->tx_buf[0] = ZIGBEE_FRAME_SOF0;
    zigbee->tx_buf[1] = ZIGBEE_FRAME_SOF1;
    zigbee->tx_buf[2] = ZIGBEE_PAYLOAD_LEN;
    data_pack(&zigbee->tx_buf[3], data);

    return HAL_UART_Transmit_DMA(&ZIGBEE_UART_HANDLE, zigbee->tx_buf, ZIGBEE_PAYLOAD_LEN + ZIGBEE_FRAME_OVERHEAD);
}

HAL_StatusTypeDef Zigbee_Receive(ZigbeeHandle_TypeDef *zigbee, ZigbeeData_TypeDef *data)
{
    if (data == NULL) return HAL_ERROR;
    if (!zigbee->rx_valid)
        return HAL_ERROR;

    *data = zigbee->rx_data;
    zigbee->rx_valid = 0U;
    return HAL_OK;
}

void Zigbee_ErrorHandler(ZigbeeHandle_TypeDef *zigbee)
{
    if (zigbee->status.state == ZIGBEE_STATE_DISCONNECTED)
        return;

    if ((HAL_GetTick() - zigbee->status.last_rx_tick) >= ZIGBEE_RX_TIMEOUT_MS)
    {
        /* 首次检测到超时：停止 DMA，标记错误 */
        if (zigbee->status.state != ZIGBEE_STATE_ERROR)
        {
            zigbee->status.state = ZIGBEE_STATE_ERROR;
            zigbee->status.error_count++;
        }
    }
}


void Zigbee_RxEventHandler(ZigbeeHandle_TypeDef *zigbee, UART_HandleTypeDef *huart, uint16_t Size)
{
    frame_parse(zigbee, zigbee->rx_dma_buf, Size);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, zigbee->rx_dma_buf, ZIGBEE_RX_BUF_SIZE);
}

void Zigbee_RxErrorHandler(ZigbeeHandle_TypeDef *zigbee, UART_HandleTypeDef *huart)
{
    /* 溢出/帧/噪声等接收错误发生时，HAL已经在HAL_UART_IRQHandler内部把DMA接收中止并将
       RxState还原为READY(见UART_EndRxTransfer+HAL_DMA_Abort_IT)，此处必须主动重新挂起，
       否则该串口会永久停止接收，之后再也不会有任何RxEvent/Error回调 */
    zigbee->status.error_count++;
    zigbee->status.state = ZIGBEE_STATE_ERROR;

    HAL_UARTEx_ReceiveToIdle_DMA(huart, zigbee->rx_dma_buf, ZIGBEE_RX_BUF_SIZE);
}
