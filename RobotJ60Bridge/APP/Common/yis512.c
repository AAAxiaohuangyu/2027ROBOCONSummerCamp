#include "yis512.h"
#include "bsp_config.h"

#include "fdcan_common.h"

static uint16_t Yis512ReadU16LE(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0]) | ((uint16_t)data[1] << 8));
}

static float Yis512ConvertEulerDeg(const uint8_t *data)
{
    return ((float)Yis512ReadU16LE(data) * YIS512_EULER_SCALE_DEG) +
           YIS512_EULER_OFFSET_DEG;
}

void Yis512Init(Yis512_TypeDef *yis512)
{
    if (yis512 == NULL)
    {
        return;
    }

    yis512->pitch_deg = 0.0f;
    yis512->roll_deg = 0.0f;
    yis512->yaw_deg = 0.0f;
    yis512->valid = 0U;
    yis512->update_cnt = 0U;
    yis512->last_update_ms = 0U;

    /* CAN 的过滤器、FIFO0 中断通知和启动统一使用 Common 的公共封装。 */
    FDCANExtendedInit(YIS512_FDCAN_HANDLE, YIS512_EULER_EXTENDED_ID);
}

uint8_t Yis512ParseEulerFrame(Yis512_TypeDef *yis512,
                               FDCAN_HandleTypeDef *hfdcan,
                               const FDCAN_RxHeaderTypeDef *rx_header,
                               const uint8_t data[8])
{
    float pitch_deg;
    float roll_deg;
    float yaw_deg;

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
    pitch_deg = Yis512ConvertEulerDeg(&data[0]);
    roll_deg = Yis512ConvertEulerDeg(&data[2]);
    yaw_deg = Yis512ConvertEulerDeg(&data[4]);

    yis512->pitch_deg = pitch_deg;
    yis512->roll_deg = roll_deg;
    yis512->yaw_deg = yaw_deg;
    yis512->last_update_ms = HAL_GetTick();
    yis512->update_cnt++;
    yis512->valid = 1U;

    return 1U;
}
