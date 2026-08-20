/*
 * yis512.h
 *
 * Yesense YIS512 的最小 CAN 欧拉角驱动。
 *
 * 本驱动只解析 YIS512 通过 SAE J1939 扩展 CAN 帧主动发送的欧拉角报文：
 * pitch、roll、yaw。首次有效报文自动定义为软件初始姿态，之后结构体中的
 * pitch_deg、roll_deg、yaw_deg 保存相对初始姿态的变化量，单位为 degree。
 *
 * 驱动不创建任务、不操作串口、不发送 CAN 配置命令。它负责配置自身的扩展帧接收过滤器，
 * 并在 BSP 的 FDCAN FIFO0 回调转发后取出 CAN 帧、完成解析。
 */
#ifndef YIS512_H
#define YIS512_H

#include <stdint.h>

#include "fdcan.h"

/*
 * 当前长门板的默认接线配置。
 * YIS512 的默认 CAN 波特率为 500 kbit/s；本工程 FDCAN2 已在 Camptest.ioc 中
 * 配置为 Classic CAN、Normal Mode、500 kbit/s。
 *
 * 迁移到其他 CAN 接口时，需要同时：
 * 1. 修改本文件中的 FDCAN 句柄与引脚宏；
 * 2. 在目标工程的 CubeMX .ioc 中配置相同的 FDCAN 外设、引脚、500 kbit/s 和扩展帧过滤器；
 * 3. 重新 Generate Code。
 * 这些宏用于集中说明和 BSP 对接；本驱动不手工改写 GPIO，以免覆盖 CubeMX 配置。
 */
#define YIS512_FDCAN_HANDLE                    (&hfdcan2)
#define YIS512_CAN_BITRATE_BPS                 (500000UL)

#define YIS512_FDCAN_RX_GPIO_PORT              GPIOB
#define YIS512_FDCAN_RX_PIN                    GPIO_PIN_12
#define YIS512_FDCAN_TX_GPIO_PORT              GPIOB
#define YIS512_FDCAN_TX_PIN                    GPIO_PIN_13
#define YIS512_FDCAN_GPIO_ALTERNATE            GPIO_AF9_FDCAN2

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
    /*
     * 对上层真正提供的姿态角，均相对本次启动后第一帧有效数据，单位为 degree。
     * 例如 yaw_deg=10.0f 表示当前相对初始朝向变化约 +10 度。
     */
    volatile float pitch_deg;
    volatile float roll_deg;
    volatile float yaw_deg;

    /* 首帧记录的绝对读数，仅供驱动内部计算相对变化量和调试查看。 */
    volatile float initial_pitch_deg;
    volatile float initial_roll_deg;
    volatile float initial_yaw_deg;

    /* 至少成功解析一帧欧拉角后为 1；update_count 每成功解析一帧加 1。 */
    volatile uint8_t valid;
    volatile uint32_t update_count;
    volatile uint32_t last_update_ms;
} Yis512_TypeDef;

/*
 * 清空相对姿态与首帧标志，并通过 fdcan_common 配置欧拉角扩展帧过滤器、启动 FDCAN。
 * 调用后，下一帧有效欧拉角会自动成为新的软件原点；不发送任何 YIS512 配置命令。
 */
void Yis512_Init(Yis512_TypeDef *yis512);

/*
 * 解析 BSP 已接收的一帧 CAN 数据。
 * 仅接受来自 YIS512_FDCAN_HANDLE、ID=YIS512_EULER_EXTENDED_ID、长度为 6 字节的扩展帧。
 * 返回 1 表示当前帧已成功更新 yis512；返回 0 表示该帧不属于 YIS512 欧拉角或格式不正确。
 */
uint8_t Yis512_ParseEulerFrame(Yis512_TypeDef *yis512,
                               FDCAN_HandleTypeDef *hfdcan,
                               const FDCAN_RxHeaderTypeDef *rx_header,
                               const uint8_t data[8]);

/*
 * 供全局 HAL_FDCAN_RxFifo0Callback 转发。
 * 驱动在内部从 FIFO0 取出一帧数据，并调用 Yis512_ParseEulerFrame() 更新结构体。
 * 回调中不做串口输出、延时或复杂计算。
 */
void Yis512_RxFifo0Callback(Yis512_TypeDef *yis512,
                             FDCAN_HandleTypeDef *hfdcan,
                             uint32_t rx_fifo0_its);

#endif /* YIS512_H */
