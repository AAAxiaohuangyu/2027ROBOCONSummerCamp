#include "bsp_callback.h"
#include "Core.h"
#include "bsp_config.h"

/* FDCAN FIFO0收到新报文:按hfdcan实例匹配到挂载在该总线上的电机/电调组,解析反馈;
   命中J60(升降)后即返回,否则再尝试M3508电调组(底盘),均先判断句柄非空且实例匹配
   再进入对应解析,避免不同子系统误解析对方的帧 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    /*
     * FIFO0 可同时接收 J60 与底盘电调反馈。先尝试按 CAN 外设实例和标准 ID
     * 匹配升降 J60；不命中时再转交 M3508。回调只做收帧和解析，不运行规划或
     * 长时间控制算法，周期发送由各自任务完成。
     */
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

/* UART空闲线收到一帧：ID 7 已替换为 PWM 舵机，GO 组只接收前后轴 ID 3 的 UART4 回包。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
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

    /* UART4上的ID3回包交给同一个GO电机组。 */
    GOM8010GroupRxEvent(&Robot.roboticarm.go_motors, huart, Size);
}
