#include "vision.h"
#include "bsp_config.h"
#include <string.h>

static uint8_t Vision_Crc8(const uint8_t *data, uint16_t length)
{
    /* CRC-8 的初值为 0xFF，多项式为 0x31。 */
    uint8_t crc = 0xFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x31U) : (uint8_t)(crc << 1U);
        }
    }
    return crc;
}

void Vision_Init(VisionHandle_TypeDef *vision)
{
    /* 主控只应在颜色不为 UNKNOWN 时使用对应的视觉结果。 */
    vision->current_colour = KFS_COLOUR_UNKNOWN;
    vision->next_colour = KFS_COLOUR_UNKNOWN;
    vision->chassis_x = 0.0f;
    vision->chassis_y = 0.0f;

    /* VISION_UART_HANDLE在CubeMX完成分配前于bsp_config.h中为NULL占位,判空后再启动接收,
       句柄补齐后无需再改这里 */
    if (VISION_UART_HANDLE != NULL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(VISION_UART_HANDLE, vision->rx_dma_buf, VISION_RX_BUF_SIZE);

        /* 禁用半传输中断，避免在帧接收到一半时触发回调 */
        __HAL_DMA_DISABLE_IT(VISION_UART_HANDLE->hdmarx, DMA_IT_HT);
    }
}

uint8_t Vision_ParseFrame(VisionHandle_TypeDef *vision, const uint8_t *frame, uint16_t size)
{
    /* 先校验帧格式和 CRC，确认完整载荷正确后才解析颜色和坐标。 */
    if (size != VISION_FRAME_SIZE || frame[0] != VISION_FRAME_SOF0 ||
        frame[1] != VISION_FRAME_SOF1 || frame[2] != VISION_FRAME_TYPE_KFS ||
        frame[3] != VISION_KFS_PAYLOAD_LEN ||
        Vision_Crc8(&frame[2], (uint16_t)(2U + VISION_KFS_PAYLOAD_LEN)) != frame[VISION_FRAME_SIZE - 1U])
    {
        return 0U;
    }

    /* 两个颜色字节都合法后再更新全部结果，避免新旧数据混用。 */
    if ((frame[4] != KFS_COLOUR_BLUE && frame[4] != KFS_COLOUR_RED) ||
        (frame[5] != KFS_COLOUR_BLUE && frame[5] != KFS_COLOUR_RED))
    {
        return 0U;
    }

    vision->current_colour = (KFS_COLOUR)frame[4];
    vision->next_colour = (KFS_COLOUR)frame[5];

    /* memcpy 避免未对齐地址的 float 指针访问；X 位于第 6 字节，Y 位于第 10 字节。 */
    memcpy(&vision->chassis_x, &frame[6], sizeof(vision->chassis_x));
    memcpy(&vision->chassis_y, &frame[10], sizeof(vision->chassis_y));
    return 1U;
}

void Vision_RxEventHandler(VisionHandle_TypeDef *vision, UART_HandleTypeDef *huart, uint16_t Size)
{
    Vision_ParseFrame(vision, vision->rx_dma_buf, Size);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, vision->rx_dma_buf, VISION_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
