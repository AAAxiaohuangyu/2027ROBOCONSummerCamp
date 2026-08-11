#ifndef __GO_M8010_H__
#define __GO_M8010_H__

#include "main.h"
#include "usart.h"
#include "StrategyAlogrithm.h"
#include <stdint.h>

#define PI2 6.28318f /* 2*PI,用于rad与rev(转)之间的单位换算 */

#define GO_M8010_REDUCTION_RATIO 6.33f /* 转子到输出端减速比,协议帧内部仍按转子侧数据收发,由驱动负责与本结构体的输出端数据互相换算 */

/* 位置控制内置速度规划(七段S曲线)默认参数,调参从这里改 */
#define GO_M8010_POS_CTRL_A_MAX 6.0f  /* 加速度上限 */
#define GO_M8010_POS_CTRL_V_MAX 3.0f  /* 速度上限 */
#define GO_M8010_POS_CTRL_J     24.0f /* 加加速度(jerk)上限 */
#define GO_M8010_POS_CTRL_KP    0.55f /* 位置环增益kp */
#define GO_M8010_POS_CTRL_KD    0.2f  /* 速度环增益kd */

#define GO_M8010_CONTROL_FRAME_SIZE  17U
#define GO_M8010_FEEDBACK_FRAME_SIZE 16U

typedef struct
{
    uint8_t bytes[GO_M8010_CONTROL_FRAME_SIZE];
} GOM8010ControlPacket_TypeDef;

typedef struct
{
    uint8_t bytes[GO_M8010_FEEDBACK_FRAME_SIZE];
} GOM8010FeedbackPacket_TypeDef;

typedef struct
{
    UART_HandleTypeDef *huart; /* 该电机挂载的RS485串口实例,支持同一份驱动挂多路总线 */
    uint8_t id;
    uint8_t mode;
    uint8_t timeout;
    float torque;   /* 输出端扭矩,驱动内部换算为转子侧后再打包 */
    float speed;    /* 输出端转速,驱动内部换算为转子侧后再打包 */
    float position; /* 输出端位置,驱动内部换算为转子侧后再打包 */
    float kp;
    float kd;
    GOM8010ControlPacket_TypeDef packet;
} GOM8010MotorCmd_TypeDef;

typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t id;
    uint8_t mode;
    uint8_t timeout;
    int8_t temp;
    uint8_t error;
    float torque;   /* 输出端扭矩,驱动内部已由转子侧换算得到 */
    float speed;    /* 输出端转速,驱动内部已由转子侧换算得到 */
    float position; /* 输出端位置,驱动内部已由转子侧换算得到 */
    uint16_t force;
    uint16_t calc_crc;
    uint32_t bad_msg;
    uint8_t valid;
    GOM8010FeedbackPacket_TypeDef packet;
} GOM8010MotorFeedback_TypeDef;

/* 初始化指令/反馈句柄;huart为该电机挂载的RS485串口实例(如&huart1) */
void GOM8010MotorInit(GOM8010MotorCmd_TypeDef *motor_c, GOM8010MotorFeedback_TypeDef *motor_f, uint8_t id, UART_HandleTypeDef *huart);

/* 打包控制帧并通过DMA发送,不阻塞调用者 */
void GOM8010MotorSendControl(GOM8010MotorCmd_TypeDef *motor);

/* 解析反馈帧:接收由上层空闲线中断驱动(HAL_UARTEx_ReceiveToIdle_IT接收至motor_r->packet.bytes,
   在HAL_UARTEx_RxEventCallback中取得Size后调用本函数),本驱动不包含中断回调 */
void GOM8010MotorParseFeedback(GOM8010MotorFeedback_TypeDef *motor_r, uint16_t size);

/* 力位速混合控制的位置环:内置速度规划(SpeedPlan),把"目标位置"闭环为torque前馈+kp/kd跟踪的position/speed指令 */
typedef struct
{
    GOM8010MotorCmd_TypeDef cmd;
    GOM8010MotorFeedback_TypeDef fb;
    SpeedPlan_TypeDef plan;
    float position_target;
} GOM8010PositionControl_TypeDef;

/* 初始化位置环:电机句柄+规划器均按GO_M8010_POS_CTRL_*默认参数初始化 */
void GOM8010PositionControlInit(GOM8010PositionControl_TypeDef *pc, uint8_t id, UART_HandleTypeDef *huart);

/* 下发新的目标位置,(重新)触发规划;运动中调用即为打断 */
void GOM8010PositionControlSetTarget(GOM8010PositionControl_TypeDef *pc, float position_target);

/* 周期调用:推进规划并发送一次控制帧;调用前需已通过GOM8010MotorParseFeedback更新pc->fb */
void GOM8010PositionControlUpdate(GOM8010PositionControl_TypeDef *pc);

#endif /* __GO_M8010_H__ */
