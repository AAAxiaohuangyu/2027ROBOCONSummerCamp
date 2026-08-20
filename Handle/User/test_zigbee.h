#ifndef __TEST_ZIGBEE_H
#define __TEST_ZIGBEE_H

#include "zigbee.h"
#include <stdint.h>

/* 供 Debug Watch 直接观察的回环测试数据 */
extern ZigbeeData_TypeDef test_tx_data; /**< 当前发送数据 */
extern ZigbeeData_TypeDef test_rx_data; /**< 最新接收数据 */

/* 测试发送周期 ms */
#define TEST_ZIGBEE_SEND_PERIOD_MS     100U

/* 测试状态 */
typedef struct
{
    uint32_t send_count;       /**< 成功启动DMA发送次数 */
    uint32_t receive_count;    /**< 接收到有效帧次数 */
    uint32_t pass_count;       /**< 回环数据匹配次数 */
    uint32_t fail_count;       /**< 回环数据不匹配次数 */
    uint32_t busy_count;       /**< DMA忙次数 */
    uint32_t error_count;      /**< 发送或接收错误次数 */
    uint32_t last_send_tick;   /**< 上一次发送时间 */
} TestZigbeeStatus_TypeDef;

/* 初始化测试 */
HAL_StatusTypeDef Test_Zigbee_Init(void);

/* 周期任务，建议在while循环中反复调用 */
void Test_Zigbee_Task(void);

/* 获取测试状态 */
const TestZigbeeStatus_TypeDef *Test_Zigbee_GetStatus(void);

#endif
