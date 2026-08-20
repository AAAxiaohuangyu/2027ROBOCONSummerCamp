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

/* 将 ZigbeeData_TypeDef 按大端字节序拼入 buf */
static void data_pack(uint8_t *buf, const ZigbeeData_TypeDef *data)
{
    uint8_t command_byte = 0U;

    /* 底盘平动x轴速度 */
    buf[0] = (uint8_t)((uint16_t)data->chassis.speed_vx >> 8U);
    buf[1] = (uint8_t)((uint16_t)data->chassis.speed_vx);

    /* 底盘平动y轴速度 */
    buf[2] = (uint8_t)((uint16_t)data->chassis.speed_vy >> 8U);
    buf[3] = (uint8_t)((uint16_t)data->chassis.speed_vy);

    /* 底盘旋转速度 */
    buf[4] = (uint8_t)((uint16_t)data->chassis.omega >> 8U);
    buf[5] = (uint8_t)((uint16_t)data->chassis.omega);

    /* 前后关节指令 */
    buf[6] = (uint8_t)((uint16_t)data->joint.front_back >> 8U);
    buf[7] = (uint8_t)((uint16_t)data->joint.front_back);

    /* 上下关节指令 */
    buf[8] = (uint8_t)((uint16_t)data->joint.up_down >> 8U);
    buf[9] = (uint8_t)((uint16_t)data->joint.up_down);

    /* 翻转关节指令 */
    buf[10] = (uint8_t)((uint16_t)data->joint.flip >> 8U);
    buf[11] = (uint8_t)((uint16_t)data->joint.flip);

    /*
     * 四个0/1指令合并到一个字节：
     * bit0：抓取
     * bit1：急停
     */
    command_byte |= (uint8_t)((data->command.grab & 0x01U) << 0U);
    command_byte |= (uint8_t)((data->command.emergency_stop & 0x01U) << 1U);

    buf[12] = command_byte;
}

/* 从 buf（19 字节）按大端字节序解析到 ZigbeeData_TypeDef */
static void data_unpack(ZigbeeData_TypeDef *data, const uint8_t *buf)
{
    /* 底盘x轴平动速度 */
    data->chassis.speed_vx =
        (int16_t)(((uint16_t)buf[0] << 8U) | buf[1]);

    /* 底盘y轴平动速度 */
    data->chassis.speed_vy =
        (int16_t)(((uint16_t)buf[2] << 8U) | buf[3]);

    /* 底盘旋转速度 */
    data->chassis.omega =
        (int16_t)(((uint16_t)buf[4] << 8U) | buf[5]);

    /* 前后关节指令 */
    data->joint.front_back =
        (int16_t)(((uint16_t)buf[6] << 8U) | buf[7]);

    /* 上下关节指令 */
    data->joint.up_down =
        (int16_t)(((uint16_t)buf[8] << 8U) | buf[9]);

    /* 翻转关节指令 */
    data->joint.flip =
        (int16_t)(((uint16_t)buf[10] << 8U) | buf[11]);
    /* 解析四个0/1指令 */
    data->command.grab = (buf[12] >> 0U) & 0x01U;
    data->command.emergency_stop = (buf[12] >> 1U) & 0x01U;
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
        zigbee->explained_data = zigbee->rx_data;
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
    memset(&zigbee->explained_data, 0, sizeof(zigbee->explained_data));
    zigbee->rx_valid = 0U;
    zigbee->status.state = ZIGBEE_STATE_DISCONNECTED;
    zigbee->status.last_rx_tick = HAL_GetTick(); /* 防止上电立即误判超时 */

    HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(&ZIGBEE_UART_HANDLE, zigbee->rx_dma_buf, ZIGBEE_RX_BUF_SIZE);

    /* 禁用半传输中断，避免在帧接收到一半时触发回调 */
    __HAL_DMA_DISABLE_IT(ZIGBEE_UART_HANDLE.hdmarx, DMA_IT_HT);

    if (ret != HAL_OK)
        zigbee->status.state = ZIGBEE_STATE_ERROR;

    return ret;
}

HAL_StatusTypeDef Zigbee_Send(ZigbeeHandle_TypeDef *zigbee, const ZigbeeData_TypeDef *data)
{
    if (data == NULL) return HAL_ERROR;

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

void Zigbee_SendAT(ZigbeeHandle_TypeDef *zigbee, const char *command)
{
    uint16_t rx_len;

    memset(zigbee->at_response, 0, sizeof(zigbee->at_response));

    /* 发送 AT 指令 */
    HAL_UART_Transmit(&ZIGBEE_UART_HANDLE, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);

    /* 接收返回消息 */
    HAL_UARTEx_ReceiveToIdle(&ZIGBEE_UART_HANDLE, zigbee->at_response, sizeof(zigbee->at_response) - 1U, &rx_len, 1000U);
}

void Zigbee_RxEventHandler(ZigbeeHandle_TypeDef *zigbee, UART_HandleTypeDef *huart, uint16_t Size)
{
    frame_parse(zigbee, zigbee->rx_dma_buf, Size);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, zigbee->rx_dma_buf, ZIGBEE_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
