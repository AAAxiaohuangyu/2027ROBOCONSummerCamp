#include "encoder_baudrate_switch.h"
#include "Core.h"
#include "cmsis_os2.h"
#include "fdcan.h"

/* FDCAN1: PA11=RX, PA12=TX；切换时仅连接一只 500 kbps 码盘。 */
/* 先配置 ID1，成功并断开后改为 ID2。 */
#define ENCODER_BAUDRATE_SWITCH_NODE_ID    (1U)
#define ENCODER_BAUDRATE_SWITCH_TIMEOUT_MS (500U)

/* Keil Watch：0=未开始，1=等待回包，2=成功，3=超时。 */
uint8_t EncoderBaudrateSwitchStatus;

uint8_t EncoderBaudrateSwitchInit(void)
{
    /* 本文件存在时总是创建一次切换任务；完成后请手动删除该测试模块。 */
    EncoderBaudrateSwitchStatus = 0U;
    return 1U;
}

void EncoderBaudrateSwitchTask(void *argument)
{
    uint32_t start_tick;

    (void)argument;
    osDelay(2000U);

    /* 经 FDCAN1 发送 04 ID 03 01，要求码盘切换为 1 Mbps。 */
    EncoderSetCanBaudRate(&Robot.encoder, &hfdcan1,
                           ENCODER_BAUDRATE_SWITCH_NODE_ID, ENCODER_CAN_BAUD_1M);

    EncoderBaudrateSwitchStatus = 1U;
    start_tick = HAL_GetTick();

    /* FIFO1 回调收到确认帧后置位 baudrate_ack。 */
    while ((Robot.encoder.baudrate_ack == 0U) &&
           ((HAL_GetTick() - start_tick) < ENCODER_BAUDRATE_SWITCH_TIMEOUT_MS))
    {
        osDelay(1U);
    }

    EncoderBaudrateSwitchStatus = (Robot.encoder.baudrate_ack != 0U) ? 2U : 3U;
    /* 只执行一次，防止重复配置。 */
    for (;;)
    {
        osDelay(osWaitForever);
    }
}
