#ifndef __ZIGBEE_H__
#define __ZIGBEE_H__

#include "main.h"
#include <stdint.h>

#define ZIGBEE_UART_HANDLE (huart1)

/* DMA 接收缓冲区大小 */
#define ZIGBEE_RX_BUF_SIZE 64U

/* 接收超时阈值 ms */
#define ZIGBEE_RX_TIMEOUT_MS 10U

/* 数据帧格式 */
#define ZIGBEE_FRAME_SOF0 0xAAU /* 帧头第一字节 */
#define ZIGBEE_FRAME_SOF1 0x55U /* 帧头第二字节 */
/** 帧固定：SOF×2 + LEN(1B) = 3 字节 */
#define ZIGBEE_FRAME_OVERHEAD 3U

/*
 * 有效载荷：
 * chassis.speed_vx        4字节
 * chassis.speed_vy        4字节
 * chassis.omega           4字节
 * joint.front_back        4字节
 * joint.up_down           4字节
 * joint.flip              4字节
 * command                 1字节
 *
 * 总计25字节
 */
#define ZIGBEE_PAYLOAD_LEN 25U

/* 用于接收AT指令初始化时的返回情况 */
#define ZIGBEE_AT_RX_SIZE 64U

typedef struct
{
    float speed_vx; /**< 平动x轴速度 */
    float speed_vy; /**< 平动y轴速度 */
    float omega;    /**< 旋转速度 */
} ZigbeeChassisCmd_TypeDef;


typedef struct
{
    float front_back; /**< 前后关节速度，正负表示方向 */
    float up_down;    /**< 上下关节速度，正负表示方向 */
    float flip;       /**< 正逆翻转速度，正负表示方向 */
} Joint_TypeDef;

/*
0:释放,1:抓取
*/
typedef struct
{
    uint8_t grab;           /**< 抓取指令 */
    uint8_t emergency_stop; /**< 急停指令 */
    uint8_t mode;           /* 模式选择，默认0 自动 */
} Command_TypeDef;

typedef struct
{
    ZigbeeChassisCmd_TypeDef chassis; /**< 底盘控制 */
    Joint_TypeDef joint;              /**< 机械臂控制 */
    Command_TypeDef command;          /**< 功能指令 */
} ZigbeeData_TypeDef;

typedef enum
{
    ZIGBEE_STATE_DISCONNECTED = 0, /**< 未连接 / 初始化后尚未收到有效帧 */
    ZIGBEE_STATE_CONNECTED,        /**< 正常通信中 */
    ZIGBEE_STATE_ERROR,            /**< 连接超时，等待重连 */
} ZigbeeState_e;

typedef struct
{
    ZigbeeState_e state;   /**< 当前连接状态 */
    uint32_t rx_count;     /**< 累计有效帧数 */
    uint32_t error_count;  /**< 累计错误次数 */
    uint32_t last_rx_tick; /**< 最后收到有效帧时的 HAL tick */
} ZigbeeStatus_TypeDef;

typedef struct
{
    uint8_t rx_dma_buf[ZIGBEE_RX_BUF_SIZE];                     /**< DMA接收缓冲 */
    uint8_t tx_buf[ZIGBEE_PAYLOAD_LEN + ZIGBEE_FRAME_OVERHEAD]; /**< 发送缓冲 */
    uint8_t at_response[ZIGBEE_AT_RX_SIZE];                     /**< AT指令返回缓冲 */
    ZigbeeData_TypeDef rx_data;                                 /**< 最新解析完成的控制帧 */
    ZigbeeData_TypeDef tx_data;                                 // 发送数据
    uint8_t rx_valid;                                           /**< 新帧就绪标志 */
    ZigbeeStatus_TypeDef status;                                /**< 连接状态与统计信息 */
    ZigbeeData_TypeDef explained_data;                          /**< 解析后数据,持续保持最新一帧,不随Zigbee_Receive消费而清空 */
} ZigbeeHandle_TypeDef;

HAL_StatusTypeDef Zigbee_Init(ZigbeeHandle_TypeDef *zigbee);
HAL_StatusTypeDef Zigbee_Send(ZigbeeHandle_TypeDef *zigbee, const ZigbeeData_TypeDef *data);
HAL_StatusTypeDef Zigbee_Receive(ZigbeeHandle_TypeDef *zigbee, ZigbeeData_TypeDef *data);

/**
 * @brief  错误处理与自动重连
 * @note   须在周期性任务中调用，建议周期 ≤ 100ms
 */
void Zigbee_ErrorHandler(ZigbeeHandle_TypeDef *zigbee);

/* 供上层在HAL_UARTEx_RxEventCallback中调用(huart匹配ZIGBEE_UART_HANDLE时):解析本次DMA空闲线
   事件收到的数据并重新挂起下一次接收;HAL回调全局唯一,不能在本文件内直接实现,由bsp_callback.c
   统一分发 */
void Zigbee_RxEventHandler(ZigbeeHandle_TypeDef *zigbee, UART_HandleTypeDef *huart, uint16_t Size);

const ZigbeeStatus_TypeDef *Zigbee_GetStatus(const ZigbeeHandle_TypeDef *zigbee);
const uint8_t *Zigbee_GetATResponse(const ZigbeeHandle_TypeDef *zigbee);

#endif
