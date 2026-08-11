#include "Test.h"
#include "cmsis_os.h"
#include "bsp_config.h"
#include <string.h>
#include <stdlib.h>

/*
S曲线速度规划(SpeedPlan)+ GO-M8010电机力位速混合控制联调测试:
    - 规划器+力位速控制已封装为GO_M8010驱动的GOM8010PositionControl_TypeDef位置环API,
      本任务只需下发目标位置并周期调用GOM8010PositionControlUpdate,由驱动内部完成规划与控制帧下发
    - position_actual取自电机RS485反馈(GOM8010MotorFeedback_TypeDef.position),闭环由真实电机完成,不再开环模拟
    - 通过DEBUG_UART以ASCII字符串(不带终止符,如仅发"1")接收上位机下发的目标距离,靠空闲线中断判定一帧结束,收到即(重新)触发一次规划,支持运动中打断测试
    - 本任务以200Hz(5ms)周期调用GOM8010PositionControlUpdate并下发一次电机控制帧;规划器内部仍按1ms子步长精细积分,精度不受调用频率影响
    - 规划状态/电机反馈按VOFA+ JustFloat协议通过DEBUG_UART发出,可在VOFA+中实时画图观察,便于kp/kd等参数调整
*/

#define GO_M8010_TEST_ID 3u /* 被测电机的RS485地址 */

#define GO_M8010_UART_HANDLE   (&huart5)
#define GO_M8010_UART_INSTANCE UART5

#define VOFA_SEND_DIVIDER 1u /* 规划器200Hz运行,每VOFA_SEND_DIVIDER个周期发送一帧数据,避免串口过载 */

#define TARGET_RX_LINE_MAX 16u /* 上位机以ASCII字符串下发目标距离,足够容纳典型数值长度(含符号/小数点) */

static uint8_t rx_line_buf[TARGET_RX_LINE_MAX]; /* 空闲线中断DMA/IT接收缓冲区 */
static volatile float target_distance;   /* 最近一次收到的目标距离 */
static volatile uint8_t new_target_flag; /* 收到新目标距离,尚未被任务消费 */

GOM8010PositionControl_TypeDef go_motor_pc;

/* 按VOFA+ JustFloat协议打包并通过DEBUG_UART发送:数据帧 = count个float(小端)+4字节帧尾 00 00 80 7F */
static void VofaJustFloatSend(const float *data, uint8_t count)
{
    uint8_t buf[64]; /* 最多支持15个float,数据量增加请相应加大缓冲区 */
    uint8_t byte_len = (uint8_t)(count * 4u);

    memcpy(buf, data, byte_len);
    buf[byte_len + 0] = 0x00;
    buf[byte_len + 1] = 0x00;
    buf[byte_len + 2] = 0x80;
    buf[byte_len + 3] = 0x7F;

    HAL_UART_Transmit_DMA(DEBUG_UART_HANDLE, buf, (uint16_t)(byte_len + 4u));
}

/* 空闲线中断触发:DEBUG_UART上位机字符串不带终止符,GO_M8010反馈帧定长,均靠总线空闲判定一帧接收完成 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == DEBUG_UART_INSTANCE)
    {
        if (Size > 0u && Size < TARGET_RX_LINE_MAX)
        {
            rx_line_buf[Size] = '\0';
            target_distance = strtof((const char *)rx_line_buf, NULL);
            new_target_flag = 1u;
        }

        HAL_UARTEx_ReceiveToIdle_IT(DEBUG_UART_HANDLE, rx_line_buf, TARGET_RX_LINE_MAX);
    }
    else if (huart->Instance == GO_M8010_UART_INSTANCE)
    {
        GOM8010MotorParseFeedback(&go_motor_pc.fb, Size);
        HAL_UARTEx_ReceiveToIdle_IT(GO_M8010_UART_HANDLE, go_motor_pc.fb.packet.bytes, GO_M8010_FEEDBACK_FRAME_SIZE);
    }
}

void StartTestTask(void *argument)
{
    GOM8010PositionControlInit(&go_motor_pc, GO_M8010_TEST_ID, GO_M8010_UART_HANDLE);

    HAL_UARTEx_ReceiveToIdle_IT(DEBUG_UART_HANDLE, rx_line_buf, TARGET_RX_LINE_MAX);
    HAL_UARTEx_ReceiveToIdle_IT(GO_M8010_UART_HANDLE, go_motor_pc.fb.packet.bytes, GO_M8010_FEEDBACK_FRAME_SIZE);

    uint32_t send_cnt = 0;

    for (;;)
    {
        if (new_target_flag)
        {
            new_target_flag = 0u;
            GOM8010PositionControlSetTarget(&go_motor_pc, target_distance); /* 收到新目标,(重新)触发规划;若正在运动中则走"打断"分支 */
        }

        float position_actual = go_motor_pc.fb.position; /* 电机RS485反馈的真实位置 */

        GOM8010PositionControlUpdate(&go_motor_pc);

        if (++send_cnt >= VOFA_SEND_DIVIDER)
        {
            send_cnt = 0;
            float vofa_data[5] = {
                go_motor_pc.plan.a,
                go_motor_pc.plan.v,
                go_motor_pc.plan.s,
                position_actual,
                go_motor_pc.position_target,
            };
            VofaJustFloatSend(vofa_data, 5u);
        }

        osDelay(5);
    }
}
