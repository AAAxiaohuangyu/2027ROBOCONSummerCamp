#ifndef __VISION_H_
#define __VISION_H_

#include "main.h"
#include <stdint.h>

#define VISION_FRAME_SOF0         (0xA5U)
#define VISION_FRAME_SOF1         (0x5AU)
#define VISION_FRAME_TYPE_KFS     (0x01U)
#define VISION_KFS_PAYLOAD_LEN    (10U)
#define VISION_FRAME_SIZE         (15U)

/* UART空闲线DMA接收缓冲区大小,与单帧定长一致 */
#define VISION_RX_BUF_SIZE        (VISION_FRAME_SIZE)

#define VISION_CORRECT_COLOUR KFS_COLOUR_BLUE //需要拿取的KFS的颜色

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
    KFS_COLOUR current_colour; /* 当前待抓取 KFS 的颜色。 */
    KFS_COLOUR next_colour;    /* 下一块待抓取 KFS 的颜色。 */
    float chassis_x;           /* 小车 x 坐标，由视觉端发送。 */
    float chassis_y;           /* 小车 y 坐标，由视觉端发送。 */
    uint8_t rx_dma_buf[VISION_RX_BUF_SIZE]; /* UART空闲线DMA接收缓冲 */
} VisionHandle_TypeDef;

/*
 * 串口帧格式：A5 5A 01 0A CURRENT_COLOUR NEXT_COLOUR X[4] Y[4] CRC8。
 * CURRENT_COLOUR 和 NEXT_COLOUR 仅可为 0（蓝）或 1（红）。
 * X、Y 均为 4 字节 IEEE-754 单精度浮点数，按小端字节序发送。
 * CRC8 从 TYPE 开始计算，覆盖 TYPE、LEN 和全部 10 字节载荷。
 */

/* 将颜色初始化为未知、坐标清零，表示尚未收到有效的视觉数据；
   VISION_UART_HANDLE在CubeMX完成分配前于bsp_config.h中为NULL占位,本函数内部
   自行判空后再启动DMA接收,句柄补齐后无需再改调用处 */
void Vision_Init(VisionHandle_TypeDef *vision);

/*
 * 解析一帧完整的视觉数据。
 * 帧头、类型、长度、CRC 和两个颜色均正确时，更新颜色及小车 x/y 坐标并返回 1；
 * 任一项错误时返回 0，保留上一次的识别结果。
 */
uint8_t Vision_ParseFrame(VisionHandle_TypeDef *vision, const uint8_t *frame, uint16_t size);

/* 供上层在HAL_UARTEx_RxEventCallback中调用(huart匹配VISION_UART_HANDLE时):解析本次DMA空闲线
   事件收到的数据并重新挂起下一次接收;HAL回调全局唯一,不能在本文件内直接实现,由bsp_callback.c
   统一分发 */
void Vision_RxEventHandler(VisionHandle_TypeDef *vision, UART_HandleTypeDef *huart, uint16_t Size);

#endif
