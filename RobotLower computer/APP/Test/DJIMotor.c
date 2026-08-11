#include "DJIMotor.h"

/* 下标1~8对应电调ID,0号不使用 */
static DJIMotor_TypeDef *motor_list[DJI_MOTOR_ID_MAX + 1] = {0};

void DJIMotorRegister(DJIMotor_TypeDef *motor, uint8_t id)
{
    if (id < DJI_MOTOR_ID_MIN || id > DJI_MOTOR_ID_MAX)
        return;

    motor->id = id;
    motor->feedback.angle = 0;
    motor->feedback.speed_rpm = 0;
    motor->feedback.current = 0;
    motor->feedback.temperature = 0;
    motor->feedback.update_cnt = 0;

    motor_list[id] = motor;
}

static void DJIMotorDecode(uint32_t std_id, uint8_t *rx_data)
{
    if (std_id <= DJI_MOTOR_FEEDBACK_ID_BASE || std_id > DJI_MOTOR_FEEDBACK_ID_BASE + DJI_MOTOR_ID_MAX)
        return;

    uint8_t id = (uint8_t)(std_id - DJI_MOTOR_FEEDBACK_ID_BASE);
    DJIMotor_TypeDef *motor = motor_list[id];
    if (motor == 0)
        return;

    motor->feedback.angle = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    motor->feedback.speed_rpm = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
    motor->feedback.current = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);
    motor->feedback.temperature = rx_data[6];
    motor->feedback.update_cnt++;
}

void DJIMotorCurrentPack(uint8_t *tx_data, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    tx_data[0] = (uint8_t)(iq1 >> 8);
    tx_data[1] = (uint8_t)(iq1);
    tx_data[2] = (uint8_t)(iq2 >> 8);
    tx_data[3] = (uint8_t)(iq2);
    tx_data[4] = (uint8_t)(iq3 >> 8);
    tx_data[5] = (uint8_t)(iq3);
    tx_data[6] = (uint8_t)(iq4 >> 8);
    tx_data[7] = (uint8_t)(iq4);
}

void DJIMotorSendCurrent(FDCAN_HandleTypeDef *FDCAN_Handle, uint16_t ctrl_id, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
    uint8_t tx_data[8];
    DJIMotorCurrentPack(tx_data, iq1, iq2, iq3, iq4);
    FDCANSendStandard(FDCAN_Handle, ctrl_id, tx_data, 8);
}

/* 覆盖HAL的弱函数,FDCANStandardInit已开启FIFO0新消息中断,收到帧后在此统一解析分发给已登记的电调 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
        return;

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
            break;

        if (RxHeader.IdType == FDCAN_STANDARD_ID)
            DJIMotorDecode(RxHeader.Identifier, RxData);
    }
}
