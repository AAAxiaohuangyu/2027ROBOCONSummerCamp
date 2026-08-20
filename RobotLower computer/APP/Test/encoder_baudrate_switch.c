#include "encoder_baudrate_switch.h"
#include "Core.h"
#include "cmsis_os2.h"

/* 每次只连接一块仍为500 kbps的BRT38M码盘，再选择SET_1M并烧录。 */
#define ENCODER_BAUDRATE_SWITCH_NONE   (0U)
#define ENCODER_BAUDRATE_SWITCH_SET_1M (1U)
#define ENCODER_BAUDRATE_SWITCH_MODE   ENCODER_BAUDRATE_SWITCH_NONE

/* 先改ID1；完成、断开后改为2U并重复烧录。 */
#define ENCODER_BAUDRATE_SWITCH_NODE_ID    (1U)
#define ENCODER_BAUDRATE_SWITCH_TIMEOUT_MS (500U)

/* 0=未启动，1=等待应答，2=成功，3=超时。Keil Watch观察此变量。 */
uint8_t EncoderBaudrateSwitchStatus;

uint8_t EncoderBaudrateSwitchInit(void)
{
#if ENCODER_BAUDRATE_SWITCH_MODE == ENCODER_BAUDRATE_SWITCH_NONE
    return 0U;
#else
    EncoderBaudrateSwitchStatus = 0U;
    return 1U;
#endif
}

void EncoderBaudrateSwitchTask(void *argument)
{
#if ENCODER_BAUDRATE_SWITCH_MODE == ENCODER_BAUDRATE_SWITCH_SET_1M
    uint32_t start_tick;
#endif

    (void)argument;
    osDelay(2000U);

#if ENCODER_BAUDRATE_SWITCH_MODE == ENCODER_BAUDRATE_SWITCH_SET_1M
    /* FDCAN3仍为500 kbps时发送04 ID 03 01；成功应答后码盘立即切至1 Mbps。 */
    EncoderSetCanBaudRate(&Robot.encoder, ENCODER_X_FDCAN_HANDLE,
                           ENCODER_BAUDRATE_SWITCH_NODE_ID, ENCODER_CAN_BAUD_1M);
    EncoderBaudrateSwitchStatus = 1U;
    start_tick = HAL_GetTick();

    while ((Robot.encoder.baudrate_ack == 0U) &&
           ((HAL_GetTick() - start_tick) < ENCODER_BAUDRATE_SWITCH_TIMEOUT_MS))
    {
        osDelay(1U);
    }

    EncoderBaudrateSwitchStatus = (Robot.encoder.baudrate_ack != 0U) ? 2U : 3U;
#else
    EncoderBaudrateSwitchStatus = 3U;
#endif

    for (;;)
    {
        osDelay(osWaitForever);
    }
}
