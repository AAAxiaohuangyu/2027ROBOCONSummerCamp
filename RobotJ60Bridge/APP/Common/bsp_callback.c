#include "bsp_callback.h"
#include "BridgeGateway.h"

/* FDCAN FIFO0收到新报文:按hfdcan实例匹配到挂载在该总线上的电机/电调组/传感器,解析反馈;
   命中J60(升降)后即返回,否则再尝试M3508电调组(底盘),最后尝试YIS512(扩展过滤器0->FIFO0),
   均先判断句柄非空且实例匹配再进入对应解析,避免不同子系统误解析对方的帧 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    BridgeGateway_CanRxEvent(hfdcan, RxFifo0ITs);
}

/* FDCAN FIFO1收到新报文:目前仅编码器(过滤器0)映射到RXFIFO1 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    (void)hfdcan;
    (void)RxFifo1ITs;
}

/* UART空闲线收到一帧:按huart实例分发。ZigBee走独立处理;forward/rotate两个GO电机可能共享
   同一路RS485(huart),该情况由GOM8010GroupRxEvent内部按其仲裁表匹配处理,本函数不关心其
   电机内部字段。HAL回调全局唯一,故所有使用该机制的模块都需经本函数分发,不能各自定义 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BridgeGateway_UartRxEvent(huart, Size);
}
