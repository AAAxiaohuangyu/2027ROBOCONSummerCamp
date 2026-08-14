#ifndef __VISION_H_
#define __VISION_H_

#include <stdint.h>
#include "main.h"

#define VISION_FRAME_SOF0 (0xA5U)
#define VISION_FRAME_SOF1 (0x5AU)
#define VISION_FRAME_TYPE_KFS (0x01U)
#define VISION_KFS_PAYLOAD_LEN (1U)
#define VISION_FRAME_SIZE (5U)

#define VISION_CORRECT_COLOUR KFS_COLOUR_BLUE // 需要拿取的颜色

/* 视觉端发送的颜色编码。 */
typedef enum
{
    KFS_COLOUR_BLUE = 0U,
    KFS_COLOUR_RED = 1U,
    KFS_COLOUR_UNKNOWN = 0xFFU
} KFS_COLOUR;

/* 提供给主控的视觉识别结果。 */
typedef struct
{
    KFS_COLOUR next_colour;             /* 下一块待抓取 KFS 的颜色。 */
    uint8_t rx_dma_buf[VISION_FRAME_SIZE]; /* 串口DMA接收缓冲,固定为一帧长度。 */
} VisionHandle_TypeDef;

/*
 * 串口帧格式：A5 5A 01 01 NEXT_COLOUR。
 * NEXT_COLOUR 仅可为 0（蓝）或 1（红）。
 */

/*
 * 将颜色初始化为未知，表示尚未收到有效的视觉数据；
 * VISION_UART_HANDLE非NULL占位时,同时挂起一次DMA空闲线接收。
 */
void Vision_Init(VisionHandle_TypeDef *vision);

/*
 * 解析一帧完整的视觉数据。
 * 帧头、类型、长度和颜色均正确时，更新主控读取的颜色字段并返回 1；
 * 任一项错误时返回 0，保留上一次的识别结果。
 */
uint8_t Vision_ParseFrame(VisionHandle_TypeDef *vision, const uint8_t *frame, uint16_t size);

/* 供上层在HAL_UARTEx_RxEventCallback中调用(huart匹配VISION_UART_HANDLE时):解析本次DMA空闲线
   事件收到的一帧数据并重新挂起下一次接收;HAL回调全局唯一,由bsp_callback.c统一分发 */
void Vision_RxEventHandler(VisionHandle_TypeDef *vision, UART_HandleTypeDef *huart, uint16_t Size);

#endif
