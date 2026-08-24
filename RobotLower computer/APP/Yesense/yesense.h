#ifndef __YESENSE_H__
#define __YESENSE_H__

#include "main.h"
#include <stdint.h>

#define YESENSE_RX_DMA_BUF_SIZE  (256U)
#define YESENSE_RX_STREAM_SIZE   (512U)

typedef struct
{
    float pitch_deg;
    float roll_deg;
    float yaw_deg;
} YesenseEuler_TypeDef;

typedef struct
{
    YesenseEuler_TypeDef euler;
    uint16_t transaction_id;
    uint32_t valid_frame_count;
    uint8_t euler_valid;
    uint8_t rx_dma_buf[YESENSE_RX_DMA_BUF_SIZE];
    uint8_t rx_stream[YESENSE_RX_STREAM_SIZE];
    uint16_t rx_stream_len;
} YesenseHandle_TypeDef;

/* Starts DMA reception. The sensor must be configured to output the standard
   YIS protocol with Euler-angle item 0x40 enabled. */
void Yesense_Init(YesenseHandle_TypeDef *yesense, UART_HandleTypeDef *huart);

/* Feeds arbitrary serial data chunks. Returns the number of valid frames. */
uint16_t Yesense_ParseBytes(YesenseHandle_TypeDef *yesense, const uint8_t *data, uint16_t size);

/* Call from the shared HAL_UARTEx_RxEventCallback for the assigned UART. */
void Yesense_RxEventHandler(YesenseHandle_TypeDef *yesense, UART_HandleTypeDef *huart, uint16_t size);

/* Returns 1 after at least one valid Euler-angle item has been received. */
uint8_t Yesense_GetEuler(const YesenseHandle_TypeDef *yesense, YesenseEuler_TypeDef *euler);

#endif
