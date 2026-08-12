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

/* 力位速混合控制的控制侧:内置速度规划(SpeedPlan),把"目标位置"闭环为torque前馈+kp/kd跟踪的position/speed指令,
   最终打包为待发送的控制帧 */
typedef struct
{
    uint8_t mode;    /* 0=锁定,1=FOC闭环(力位速混合控制,常用模式),2=编码器校准 */
    uint8_t timeout;
    float torque;   /* 输出端扭矩,驱动内部换算为转子侧后再打包 */
    float speed;    /* 输出端转速,驱动内部换算为转子侧后再打包 */
    float position; /* 输出端位置,驱动内部换算为转子侧后再打包 */
    float kp;
    float kd;
    SpeedPlan_TypeDef plan;
    float position_target; /* 输出端目标位置 */
    GOM8010ControlPacket_TypeDef packet;
} GOM8010Control_TypeDef;

typedef struct
{
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
} GOM8010Feedback_TypeDef;

typedef struct
{
    uint8_t id;                 /* 电机RS485地址 */
    UART_HandleTypeDef *huart;  /* 该电机挂载的RS485串口实例,支持同一份驱动挂多路总线 */
    GOM8010Control_TypeDef control;
    GOM8010Feedback_TypeDef feedback;
} GOM8010Motor_TypeDef;

/* 初始化电机:control(含plan、按GO_M8010_POS_CTRL_*默认参数)、feedback均在此一并初始化 */
void GOM8010MotorInit(GOM8010Motor_TypeDef *motor, uint8_t id, UART_HandleTypeDef *huart);

/* 下发新的目标位置,(重新)触发规划;运动中调用即为打断 */
void GOM8010MotorSetTarget(GOM8010Motor_TypeDef *motor, float position_target);

/* 解析反馈帧:接收由上层空闲线中断驱动(HAL_UARTEx_ReceiveToIdle_IT接收至motor->feedback.packet.bytes,
   在HAL_UARTEx_RxEventCallback中取得Size后调用本函数),本驱动不包含中断回调 */
void GOM8010MotorParseFeedback(GOM8010Motor_TypeDef *motor, uint16_t size);

/* 周期调用:推进规划并发送一次控制帧;调用前需已通过GOM8010MotorParseFeedback更新motor->feedback */
void GOM8010MotorUpdate(GOM8010Motor_TypeDef *motor);

#endif /* __GO_M8010_H__ */
