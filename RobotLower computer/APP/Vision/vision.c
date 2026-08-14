#include "vision.h"
#include "bsp_config.h"
#include "usart.h"

void Vision_Init(VisionHandle_TypeDef *vision)
{
    /* 主控只应在颜色不为 UNKNOWN 时使用识别结果。 */
    vision->next_colour = KFS_COLOUR_UNKNOWN;

    /* VISION_UART_HANDLE在CubeMX完成分配前于bsp_config.h中为NULL占位,判空后再启动接收,
       句柄补齐后无需再改这里 */
    if (VISION_UART_HANDLE == NULL)
    {
        return;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(VISION_UART_HANDLE, vision->rx_dma_buf, VISION_FRAME_SIZE);
}

uint8_t Vision_ParseFrame(VisionHandle_TypeDef *vision, const uint8_t *frame, uint16_t size)
{
    /* 先校验帧格式，确认载荷长度正确后才访问颜色字节。 */
    if (size != VISION_FRAME_SIZE || frame[0] != VISION_FRAME_SOF0 ||
        frame[1] != VISION_FRAME_SOF1 || frame[2] != VISION_FRAME_TYPE_KFS ||
        frame[3] != VISION_KFS_PAYLOAD_LEN)
    {
        return 0U;
    }

    /* 颜色合法后再更新结果，避免出现无效的颜色数据。 */
    if (frame[4] != KFS_COLOUR_BLUE && frame[4] != KFS_COLOUR_RED)
    {
        return 0U;
    }

    vision->next_colour = (KFS_COLOUR)frame[4];
    return 1U;
}

void Vision_RxEventHandler(VisionHandle_TypeDef *vision, UART_HandleTypeDef *huart, uint16_t Size)
{
    Vision_ParseFrame(vision, vision->rx_dma_buf, Size);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, vision->rx_dma_buf, VISION_FRAME_SIZE);
}
