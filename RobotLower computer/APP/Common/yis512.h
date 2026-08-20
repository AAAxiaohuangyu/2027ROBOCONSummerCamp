/*
 * yis512.h
 *
 * Yesense YIS512 的最小 CAN 欧拉角驱动。
 *
 * 本驱动只解析 YIS512 通过 SAE J1939 扩展 CAN 帧主动发送的欧拉角报文：
 * pitch、roll、yaw，结构体中的 pitch_deg、roll_deg、yaw_deg 即为传感器输出的
 * 绝对读数，单位为 degree。
 *
 * 驱动不创建任务、不操作串口、不发送 CAN 配置命令，仅负责配置自身的扩展帧接收过滤器。
 * 与 J60/M3508/Encoder 等驱动一致，CAN 帧的接收统一由 bsp_callback.c 中全局唯一的
 * HAL_FDCAN_RxFifo0Callback 取出后转发，本驱动只提供 Yis512ParseEulerFrame 完成解析，
 * 不在驱动内部重复调用 HAL_FDCAN_GetRxMessage。
 */
#ifndef YIS512_H
#define YIS512_H

#include <stdint.h>

#include "fdcan.h"

/* YIS512 欧拉角报文：J1939 PGN=0xF029，源地址=0x59，使用 29 位扩展 ID。 */
#define YIS512_EULER_EXTENDED_ID               (0x0CF02959UL)
#define YIS512_EULER_DATA_LENGTH               (FDCAN_DLC_BYTES_6)

/*
 * YIS512 欧拉角的原始数据换算参数。
 * 每轴为 uint16 小端数据：angle_deg = raw x 0.0078125 - 250.0。
 */
#define YIS512_EULER_SCALE_DEG                 (0.0078125f)
#define YIS512_EULER_OFFSET_DEG                (-250.0f)

typedef struct
{
    /* 传感器输出的姿态角绝对读数，单位为 degree。 */
    volatile float pitch_deg;
    volatile float roll_deg;
    volatile float yaw_deg;

    /* 至少成功解析一帧欧拉角后为 1；update_cnt 每成功解析一帧加 1，可用于判断传感器是否离线。 */
    volatile uint8_t valid;
    volatile uint32_t update_cnt;
    volatile uint32_t last_update_ms;
} Yis512_TypeDef;

/*
 * 清空姿态角与 valid 标志，并通过 fdcan_common 配置欧拉角扩展帧过滤器、启动 FDCAN；
 * 不发送任何 YIS512 配置命令。
 */
void Yis512Init(Yis512_TypeDef *yis512);

/*
 * 解析一帧 BSP 已通过 HAL_FDCAN_GetRxMessage 取出的 CAN 数据到 yis512：
 * 仅接受来自 YIS512_FDCAN_HANDLE、ID=YIS512_EULER_EXTENDED_ID、长度为 6 字节的扩展帧，
 * 其余帧不作任何处理。返回 1 表示当前帧已成功更新 yis512；返回 0 表示该帧不属于
 * YIS512 欧拉角或格式不正确。调用者在 HAL_FDCAN_RxFifo0Callback 中取得 rx_header 和
 * data 后调用，可依次对多个模块尝试直至命中。
 */
uint8_t Yis512ParseEulerFrame(Yis512_TypeDef *yis512,
                               FDCAN_HandleTypeDef *hfdcan,
                               const FDCAN_RxHeaderTypeDef *rx_header,
                               const uint8_t data[8]);

#endif /* YIS512_H */
