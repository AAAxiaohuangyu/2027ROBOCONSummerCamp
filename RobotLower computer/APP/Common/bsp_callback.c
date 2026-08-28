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

    if (Robot.roboticarm.lift_motor.FDCAN_Handle != NULL &&
        hfdcan->Instance == Robot.roboticarm.lift_motor.FDCAN_Handle->Instance &&
        J60MotorParseFeedback(&Robot.roboticarm.lift_motor, rx_header.Identifier, rx_data))
    {
        return;
    }

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

/* UART空闲线或DMA满缓冲收到数据后按huart实例分发。GO电机使用32字节Receive-to-Idle DMA，
   在回调中扫描帧头和CRC均正确的16字节反馈帧；ZigBee/Vision仍按各自协议解析变长数据。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (Robot.roboticarm.forward_motor.huart != NULL &&
        huart->Instance == Robot.roboticarm.forward_motor.huart->Instance)
    {
        GOM8010MotorRxEvent(&Robot.roboticarm.forward_motor, huart, Size);
        return;
    }

    if (huart->Instance == ZIGBEE_UART_HANDLE.Instance)
    {
        Zigbee_RxEventHandler(&Robot.zigbee, huart, Size);
        return;
    }

    if (VISION_UART_HANDLE != NULL && huart->Instance == VISION_UART_HANDLE->Instance)
    {
        Vision_RxEventHandler(&Robot.vision, huart, Size);
        return;
    }
}

/* UART接收出现溢出/帧/噪声等错误:DMA接收模式下HAL会中止当前DMA接收并将RxState还原为READY。
   ZigBee/Vision和GO电机均在此立即重新挂起DMA接收。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == ZIGBEE_UART_HANDLE.Instance)
    {
        Zigbee_RxErrorHandler(&Robot.zigbee, huart);
        return;
    }

    if (VISION_UART_HANDLE != NULL && huart->Instance == VISION_UART_HANDLE->Instance)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, Robot.vision.rx_dma_buf, VISION_RX_BUF_SIZE);
        return;
    }

    GOM8010MotorRxErrorEvent(&Robot.roboticarm.forward_motor, huart);
}
