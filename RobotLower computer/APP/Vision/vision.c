#include "vision.h"
#include "bsp_config.h"
#include <string.h>

void Vision_Init(VisionHandle_TypeDef *vision)
{
    vision->chassis_x = 0.0f;
    vision->chassis_y = 0.0f;

    /* VISION_UART_HANDLE在CubeMX完成分配前于bsp_config.h中为NULL占位,判空后再启动接收,
       句柄补齐后无需再改这里 */
    if (VISION_UART_HANDLE != NULL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(VISION_UART_HANDLE, vision->rx_dma_buf, VISION_RX_BUF_SIZE);
    }
}

uint8_t Vision_ParseFrame(VisionHandle_TypeDef *vision, const uint8_t *frame, uint16_t size)
{
    if(frame[0] == 0xA5 && frame[1] == 0x5A && frame[2] == 0x0B)
    {
        memcpy(&vision->chassis_x, &frame[5], sizeof(vision->chassis_x));
        memcpy(&vision->chassis_y, &frame[9], sizeof(vision->chassis_y));
        vision->KFS_DIFF = frame[13];
    }
    return 1U;
}

void Vision_RxEventHandler(VisionHandle_TypeDef *vision, UART_HandleTypeDef *huart, uint16_t Size)
{
    Vision_ParseFrame(vision, vision->rx_dma_buf, Size);

    HAL_UARTEx_ReceiveToIdle_DMA(huart, vision->rx_dma_buf, VISION_RX_BUF_SIZE);
}
