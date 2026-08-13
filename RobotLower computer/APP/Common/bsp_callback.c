#include "bsp_callback.h"
#include "Core.h"

/* FDCAN FIFO0收到新报文:按hfdcan实例匹配到挂载在该总线上的电机/电调组,解析反馈;
   命中J60(升降)后即返回,否则再尝试M3508电调组(底盘),避免不同子系统误解析对方的帧 */
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
    }
}

/* UART空闲线收到一帧:按huart实例分发给ZigBee或对应的GO电机(前后平移/自转),解析反馈后
   按各自驱动的接收方式重新挂起下一次接收;HAL回调全局唯一,故所有使用该机制的模块都需
   经本函数分发,不能各自定义 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == ZIGBEE_UART_HANDLE.Instance)
    {
        Zigbee_RxEventHandler(&Robot.zigbee, huart, Size);
        return;
    }

    if (Robot.roboticarm.forward_motor.huart != NULL &&
        huart->Instance == Robot.roboticarm.forward_motor.huart->Instance)
    {
        GOM8010MotorParseFeedback(&Robot.roboticarm.forward_motor, Size);
        HAL_UARTEx_ReceiveToIdle_IT(huart, Robot.roboticarm.forward_motor.feedback.packet.bytes, GO_M8010_FEEDBACK_FRAME_SIZE);
        return;
    }

    if (Robot.roboticarm.rotate_motor.huart != NULL &&
        huart->Instance == Robot.roboticarm.rotate_motor.huart->Instance)
    {
        GOM8010MotorParseFeedback(&Robot.roboticarm.rotate_motor, Size);
        HAL_UARTEx_ReceiveToIdle_IT(huart, Robot.roboticarm.rotate_motor.feedback.packet.bytes, GO_M8010_FEEDBACK_FRAME_SIZE);
        return;
    }
}
