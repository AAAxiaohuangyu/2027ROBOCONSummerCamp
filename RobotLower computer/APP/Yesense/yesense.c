#include "yesense.h"
#include <string.h>

#define YESENSE_SOF0             (0x59U)
#define YESENSE_SOF1             (0x53U)
#define YESENSE_FRAME_OVERHEAD   (7U)
#define YESENSE_EULER_ID         (0x40U)
#define YESENSE_EULER_DATA_LEN   (12U)
#define YESENSE_SCALE_DEG        (0.000001f)

static int32_t Yesense_ReadLeI32(const uint8_t *data)
{
    return (int32_t)((uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
                     ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U));
}

static uint16_t Yesense_Checksum(const uint8_t *data, uint16_t len)
{
    uint8_t ck1 = 0U;
    uint8_t ck2 = 0U;
    uint16_t i;

    for (i = 0U; i < len; ++i)
    {
        ck1 = (uint8_t)(ck1 + data[i]);
        ck2 = (uint8_t)(ck2 + ck1);
    }
    return (uint16_t)(((uint16_t)ck2 << 8U) | ck1);
}

static uint8_t Yesense_ParseFrame(YesenseHandle_TypeDef *yesense, const uint8_t *frame, uint16_t frame_len)
{
    uint16_t payload_len = frame[4];
    uint16_t pos = 5U;
    uint16_t payload_end = (uint16_t)(5U + payload_len);
    uint16_t received_checksum;
    uint8_t euler_found = 0U;

    if (frame_len != (uint16_t)(payload_len + YESENSE_FRAME_OVERHEAD))
        return 0U;

    received_checksum = (uint16_t)frame[payload_end] | ((uint16_t)frame[payload_end + 1U] << 8U);
    if (Yesense_Checksum(&frame[2], (uint16_t)(payload_len + 3U)) != received_checksum)
        return 0U;

    while (pos < payload_end)
    {
        uint8_t id;
        uint8_t data_len;

        if ((uint16_t)(payload_end - pos) < 2U)
            return 0U;

        id = frame[pos++];
        data_len = frame[pos++];
        if (data_len > (uint16_t)(payload_end - pos))
            return 0U;

        if (id == YESENSE_EULER_ID && data_len == YESENSE_EULER_DATA_LEN)
        {
            yesense->euler.pitch_deg = (float)Yesense_ReadLeI32(&frame[pos]) * YESENSE_SCALE_DEG;
            yesense->euler.roll_deg = (float)Yesense_ReadLeI32(&frame[pos + 4U]) * YESENSE_SCALE_DEG;
            yesense->euler.yaw_deg = (float)Yesense_ReadLeI32(&frame[pos + 8U]) * YESENSE_SCALE_DEG;
            euler_found = 1U;
        }
        pos = (uint16_t)(pos + data_len);
    }

    yesense->transaction_id = (uint16_t)frame[2] | ((uint16_t)frame[3] << 8U);
    yesense->valid_frame_count++;
    if (euler_found != 0U)
        yesense->euler_valid = 1U;
    return 1U;
}

void Yesense_Init(YesenseHandle_TypeDef *yesense, UART_HandleTypeDef *huart)
{
    memset(yesense, 0, sizeof(*yesense));
    if (huart != NULL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, yesense->rx_dma_buf, YESENSE_RX_DMA_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

uint16_t Yesense_ParseBytes(YesenseHandle_TypeDef *yesense, const uint8_t *data, uint16_t size)
{
    uint16_t frame_count = 0U;

    if (data == NULL || size == 0U)
        return 0U;

    if (size > (uint16_t)(YESENSE_RX_STREAM_SIZE - yesense->rx_stream_len))
    {
        /* Retain only enough newest data to resynchronise on a split SOF. */
        yesense->rx_stream_len = 0U;
    }

    memcpy(&yesense->rx_stream[yesense->rx_stream_len], data, size);
    yesense->rx_stream_len = (uint16_t)(yesense->rx_stream_len + size);

    while (yesense->rx_stream_len >= YESENSE_FRAME_OVERHEAD)
    {
        uint16_t frame_len;

        if (yesense->rx_stream[0] != YESENSE_SOF0 || yesense->rx_stream[1] != YESENSE_SOF1)
        {
            memmove(yesense->rx_stream, &yesense->rx_stream[1], --yesense->rx_stream_len);
            continue;
        }

        frame_len = (uint16_t)(yesense->rx_stream[4] + YESENSE_FRAME_OVERHEAD);
        if (frame_len > YESENSE_RX_STREAM_SIZE)
        {
            memmove(yesense->rx_stream, &yesense->rx_stream[1], --yesense->rx_stream_len);
            continue;
        }
        if (yesense->rx_stream_len < frame_len)
            break;

        if (Yesense_ParseFrame(yesense, yesense->rx_stream, frame_len) != 0U)
            frame_count++;

        yesense->rx_stream_len = (uint16_t)(yesense->rx_stream_len - frame_len);
        if (yesense->rx_stream_len != 0U)
            memmove(yesense->rx_stream, &yesense->rx_stream[frame_len], yesense->rx_stream_len);
    }
    return frame_count;
}

void Yesense_RxEventHandler(YesenseHandle_TypeDef *yesense, UART_HandleTypeDef *huart, uint16_t size)
{
    if (size <= YESENSE_RX_DMA_BUF_SIZE)
        Yesense_ParseBytes(yesense, yesense->rx_dma_buf, size);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, yesense->rx_dma_buf, YESENSE_RX_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

uint8_t Yesense_GetEuler(const YesenseHandle_TypeDef *yesense, YesenseEuler_TypeDef *euler)
{
    if (yesense->euler_valid == 0U || euler == NULL)
        return 0U;

    *euler = yesense->euler;
    return 1U;
}
