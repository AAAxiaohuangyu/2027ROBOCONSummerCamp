#include "bsp_callback.h"
#include "Core.h"
#include "bsp_config.h"

/* FDCAN FIFO0收到新报文:按hfdcan实例匹配到挂载在该总线上的电机/电调组/传感器,解析反馈;
   命中J60(升降)后即返回,否则再尝试M3508电调组(底盘),最后尝试YIS512(扩展过滤器0->FIFO0),
   均先判断句柄非空且实例匹配再进入对应解析,避免不同子系统误解析对方的帧 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
        return;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        return;

    if (Robot.chassis.drive.motor_group.FDCAN_Handle != NULL &&
        hfdcan->Instance == Robot.chassis.drive.motor_group.FDCAN_Handle->Instance)
    {
        M3508GroupParseFeedback(&Robot.chassis.drive.motor_group, rx_header.Identifier, rx_data);
        return;
    }

    if (hfdcan->Instance == YIS512_FDCAN_HANDLE->Instance &&
        Yis512ParseEulerFrame(&Robot.yis512, hfdcan, &rx_header, rx_data))
    {
        return;
    }
}

/* FDCAN FIFO1收到新报文:目前仅编码器(过滤器0)映射到RXFIFO1 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0U)
        return;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
        return;

    if ((Robot.encoder.x_axis.FDCAN_Handle != NULL &&
         hfdcan->Instance == Robot.encoder.x_axis.FDCAN_Handle->Instance) ||
        (Robot.encoder.y_axis.FDCAN_Handle != NULL &&
         hfdcan->Instance == Robot.encoder.y_axis.FDCAN_Handle->Instance))
    {
        EncoderParseFeedback(&Robot.encoder, hfdcan, rx_header.Identifier, rx_data);
    }
}

/* UART空闲线收到一帧:按huart实例分发。ZigBee走独立处理;forward/rotate两个GO电机可能共享
   同一路RS485(huart),该情况由GOM8010GroupRxEvent内部按其仲裁表匹配处理,本函数不关心其
   电机内部字段。HAL回调全局唯一,故所有使用该机制的模块都需经本函数分发,不能各自定义 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == ZIGBEE_UART_HANDLE.Instance)
    {
        Zigbee_RxEventHandler(&Robot.zigbee, huart, Size);
        return;
    }

    /* 新增板将 J60 的 CAN 反馈封装为串口帧回传，解析后仍复用原 J60 反馈处理函数。 */
    if (huart->Instance == huart1.Instance)
    {
        J60UartBridge_RxEventHandler(&Robot.j60_bridge, huart, Size, &Robot.roboticarm.lift_motor);
        return;
    }

    GOM8010GroupRxEvent(&Robot.roboticarm.go_motors, huart, Size);
}
