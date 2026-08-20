/*
 * yis512.c
 *
 * 仅包含 YIS512 欧拉角的报文校验、字节序解析和相对初始姿态计算。
 */
#include "yis512.h"

#include "fdcan_common.h"

static uint16_t Yis512_ReadU16LE(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0]) | ((uint16_t)data[1] << 8));
}

static float Yis512_ConvertEulerDeg(const uint8_t *data)
{
    return ((float)Yis512_ReadU16LE(data) * YIS512_EULER_SCALE_DEG) +
           YIS512_EULER_OFFSET_DEG;
}

void Yis512_Init(Yis512_TypeDef *yis512)
{
    if (yis512 == NULL)
    {
        return;
    }

    yis512->pitch_deg = 0.0f;
    yis512->roll_deg = 0.0f;
    yis512->yaw_deg = 0.0f;
    yis512->initial_pitch_deg = 0.0f;
    yis512->initial_roll_deg = 0.0f;
    yis512->initial_yaw_deg = 0.0f;
    yis512->valid = 0U;
    yis512->update_count = 0U;
    yis512->last_update_ms = 0U;

    /* CAN 的过滤器、FIFO0 中断通知和启动统一使用 Common 的公共封装。 */
    FDCANExtendedInit(YIS512_FDCAN_HANDLE, YIS512_EULER_EXTENDED_ID);
}

uint8_t Yis512_ParseEulerFrame(Yis512_TypeDef *yis512,
                               FDCAN_HandleTypeDef *hfdcan,
                               const FDCAN_RxHeaderTypeDef *rx_header,
                               const uint8_t data[8])
{
    float pitch_absolute_deg;
    float roll_absolute_deg;
    float yaw_absolute_deg;

    if ((yis512 == NULL) || (hfdcan != YIS512_FDCAN_HANDLE) ||
        (rx_header == NULL) || (data == NULL))
    {
        return 0U;
    }

    /* YIS512 欧拉角帧固定使用 29 位扩展 ID，且 CAN 数据区为 3 个 uint16，共 6 字节。 */
    if ((rx_header->IdType != FDCAN_EXTENDED_ID) ||
        (rx_header->Identifier != YIS512_EULER_EXTENDED_ID) ||
        (rx_header->DataLength != YIS512_EULER_DATA_LENGTH))
    {
        return 0U;
    }

    /* 手册定义的数据顺序是 pitch、roll、yaw，每轴均为 Intel（小端）格式。 */
    pitch_absolute_deg = Yis512_ConvertEulerDeg(&data[0]);
    roll_absolute_deg = Yis512_ConvertEulerDeg(&data[2]);
    yaw_absolute_deg = Yis512_ConvertEulerDeg(&data[4]);

    /* 首帧只建立软件初始姿态，因此此时对外输出的相对角度恰好为 0。 */
    if (yis512->valid == 0U)
    {
        yis512->initial_pitch_deg = pitch_absolute_deg;
        yis512->initial_roll_deg = roll_absolute_deg;
        yis512->initial_yaw_deg = yaw_absolute_deg;
    }

    /* 结构体中的三个欧拉角始终是相对初始值的变化量，而不是传感器原始绝对读数。 */
    yis512->pitch_deg = pitch_absolute_deg - yis512->initial_pitch_deg;
    yis512->roll_deg = roll_absolute_deg - yis512->initial_roll_deg;
    yis512->yaw_deg = yaw_absolute_deg - yis512->initial_yaw_deg;
    yis512->last_update_ms = HAL_GetTick();
    yis512->update_count++;
    yis512->valid = 1U;

    return 1U;
}

void Yis512_RxFifo0Callback(Yis512_TypeDef *yis512,
                             FDCAN_HandleTypeDef *hfdcan,
                             uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];

    if ((yis512 == NULL) || (hfdcan != YIS512_FDCAN_HANDLE) ||
        ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
    {
        return;
    }

    /* FIFO0 中只取一帧；具体 ID、长度与数据内容由 ParseEulerFrame 统一校验。 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK)
    {
        return;
    }

    (void)Yis512_ParseEulerFrame(yis512, hfdcan, &rx_header, data);
}
