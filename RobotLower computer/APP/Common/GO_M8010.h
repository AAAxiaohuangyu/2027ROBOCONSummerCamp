#ifndef __GO_M8010_H__
#define __GO_M8010_H__

#include "main.h"
#include "usart.h"
#include "StrategyAlogrithm.h"
#include "bsp_config.h"
#include <stdint.h>

#define PI2 (2.0f * BSP_PI) /* 2*PI,用于rad与rev(转)之间的单位换算,PI统一取自bsp_config.h的BSP_PI */

#define GO_M8010_REDUCTION_RATIO 6.33f /* 转子到输出端减速比,协议帧内部仍按转子侧数据收发,由驱动负责与本结构体的输出端数据互相换算 */

/* 位置控制内置速度规划(七段S曲线)默认参数,调参从这里改 */
#define GO_M8010_POS_CTRL_A_MAX 120.0f  /* 加速度上限 */
#define GO_M8010_POS_CTRL_V_MAX 25.0f  /* 速度上限：由 40 降至 25，减慢伸缩机构的位置模式最高速度。 */
#define GO_M8010_POS_CTRL_J     600.0f /* 加加速度(jerk)上限 */
#define GO_M8010_POS_CTRL_KP    0.55f /* 位置环增益kp */
#define GO_M8010_POS_CTRL_KD    0.2f  /* 速度环增益kd */

/* 定速模式默认参数:不做位置跟踪(kp=0),速度环增益沿用位置模式的kd */
#define GO_M8010_VEL_CTRL_KP 0.0f
#define GO_M8010_VEL_CTRL_KD 0.3f

#define GO_M8010_CONTROL_FRAME_SIZE  17U
#define GO_M8010_FEEDBACK_FRAME_SIZE 16U

/* RS485总线共享时,一笔请求-应答事务的应答超时门限:超过此时长未收到应答,视为本电机这一笔
   丢失,总线让给下一个电机,避免某个电机断线/不应答拖死总线上的其他电机;数值按实测调整 */
#define GO_M8010_BUS_TIMEOUT_MS 20U

/* 一个GOM8010Group_TypeDef最多容纳的电机数,按需调整 */
#define GOM8010_GROUP_MAX_MOTORS 4U

typedef struct
{
    uint8_t bytes[GO_M8010_CONTROL_FRAME_SIZE];
} GOM8010ControlPacket_TypeDef;

typedef struct
{
    uint8_t bytes[GO_M8010_FEEDBACK_FRAME_SIZE];
} GOM8010FeedbackPacket_TypeDef;

typedef enum
{
    GOM8010_CTRL_MODE_POSITION = 0, /* 原控制模式:S速度规划闭环目标位置 */
    GOM8010_CTRL_MODE_VELOCITY = 1, /* 定速模式:直接跟踪目标速度,kp=0 */
} GOM8010CtrlMode_TypeDef;

typedef struct
{
    float a_max;
    float v_max;
    float j;
    float kp;
    float kd;
} GOM8010PositionCtrlParam_TypeDef;

typedef struct
{
    float kp;
    float kd;
} GOM8010VelocityCtrlParam_TypeDef;

/* 力位速混合控制的控制侧:内置速度规划(SpeedPlan),把"目标位置"闭环为torque前馈+kp/kd跟踪的position/speed指令,
   最终打包为待发送的控制帧。驱动内部使用,外部通过GOM8010Group*接口访问,不要直接操作 */
typedef struct
{
    uint8_t mode;    /* 0=锁定,1=FOC闭环(力位速混合控制,常用模式),2=编码器校准;硬件模式,由驱动固定为1,与ctrl_mode无关 */
    uint8_t timeout;
    float torque;   /* 输出端扭矩,驱动内部换算为转子侧后再打包 */
    float speed;    /* 输出端转速,驱动内部换算为转子侧后再打包 */
    float position; /* 输出端位置,驱动内部换算为转子侧后再打包 */
    float kp;
    float kd;
    SpeedPlan_TypeDef plan;
    float position_target;     /* 输出端目标位置,模式0使用 */
    float velocity_target;     /* 输出端目标转速,模式1使用 */
    float torque_feedforward;  /* 输出端前馈扭矩,由上层指定,叠加到kp/kd跟踪输出上 */
    GOM8010CtrlMode_TypeDef ctrl_mode; /* 控制算法模式:0=位置(S规划) 1=定速,与硬件mode字段区分 */
    GOM8010PositionCtrlParam_TypeDef position_param;
    GOM8010VelocityCtrlParam_TypeDef velocity_param;
    GOM8010ControlPacket_TypeDef packet;
} GOM8010Control_TypeDef;

/* 反馈数据,可通过GOM8010Group_TypeDef::motors[index].feedback只读访问,不要写 */
typedef struct
{
    uint8_t mode;
    uint8_t timeout;
    int8_t temp;
    uint8_t error;
    float torque;   /* 输出端扭矩,驱动内部已由转子侧换算得到 */
    float speed;    /* 输出端转速,驱动内部已由转子侧换算得到 */
    float position; /* 输出端位置,驱动内部已由转子侧换算得到,并已减去开机后第一帧位置(position_offset),即以上电时的位置为软件零点 */
    uint16_t force;
    uint16_t calc_crc;
    uint32_t bad_msg;
    uint8_t valid;
    GOM8010FeedbackPacket_TypeDef packet;

    float position_offset; /* 开机后第一帧的原始位置,作为软件零点基准,由update_cnt==0时捕获 */
    uint32_t update_cnt;    /* 累计解析成功的反馈帧次数,可用于判断电机是否离线 */
} GOM8010Feedback_TypeDef;

/* 单个电机:仅保存协议地址、挂载的串口与控制/反馈数据,不含总线仲裁状态(仲裁状态集中存在
   GOM8010Group_TypeDef::arbiter里)。control须通过GOM8010Group*接口修改,不要直接操作;
   feedback只读访问,不要写 */
typedef struct
{
    uint8_t id;                /* 电机RS485地址 */
    UART_HandleTypeDef *huart; /* 该电机挂载的RS485串口实例,组内所有电机须为同一实例(半双工共享总线) */
    GOM8010Control_TypeDef control;
    GOM8010Feedback_TypeDef feedback;
} GOM8010Motor_TypeDef;

/* 半双工RS485总线上组内电机共享同一huart,同一时刻只允许一笔请求-应答事务在途:
   本结构体就是这条总线的仲裁表,组内所有电机共用同一份 */
typedef struct
{
    GOM8010Motor_TypeDef *pending_motor; /* 总线上当前等待应答的电机,NULL表示总线空闲 */
    GOM8010Motor_TypeDef *last_motor;    /* 上一次成功发起请求的电机,用于多电机轮转 */
    uint32_t request_tick;               /* 发起请求时的HAL_GetTick(),用于应答超时判定 */
    uint8_t motor_count;                 /* 共享这条总线的电机数,>1时才需要轮转避让 */
} GOM8010BusArbiter_TypeDef;

/* GO电机组:电机数组 + 这条总线的仲裁表,是本驱动对外暴露的唯一顶层类型。组内所有电机共享
   同一路RS485总线(半双工,同一huart) */
typedef struct
{
    GOM8010Motor_TypeDef motors[GOM8010_GROUP_MAX_MOTORS];
    GOM8010BusArbiter_TypeDef arbiter;
    uint8_t motor_count;
} GOM8010Group_TypeDef;

/* 初始化一个空电机组,需在GOM8010GroupAddMotor之前调用 */
void GOM8010GroupInit(GOM8010Group_TypeDef *group);

/* 向电机组添加一个电机(control含plan、按GO_M8010_POS_CTRL_*默认参数,feedback均在此一并初始化),
   返回其在组内的下标(后续GOM8010GroupSetTarget等接口按此下标操作),组已满返回0xFF。
   组内所有电机须挂在同一路RS485总线(同一huart实例)上,共享同一份仲裁表 */
uint8_t GOM8010GroupAddMotor(GOM8010Group_TypeDef *group, uint8_t id, UART_HandleTypeDef *huart);

/* 下发新的目标位置,切换为模式0(位置模式)并(重新)触发规划;运动中调用即为打断 */
void GOM8010GroupSetTarget(GOM8010Group_TypeDef *group, uint8_t index, float position_target);

/* 下发新的目标速度,切换为模式1(定速模式):不做位置规划,kp=0,仅由kd跟踪目标速度 */
void GOM8010GroupSetVelocityTarget(GOM8010Group_TypeDef *group, uint8_t index, float velocity_target);

/* 下发前馈扭矩(输出端),叠加到kp/kd跟踪规划轨迹的输出上;默认0 */
void GOM8010GroupSetTorqueFeedforward(GOM8010Group_TypeDef *group, uint8_t index, float torque_feedforward);

/* 周期调用:推进组内每个电机的规划并尝试发送一次控制帧(受各自所属总线仲裁,总线忙时本周期不
   实际发送);调用前需已通过GOM8010GroupRxEvent更新过反馈 */
void GOM8010GroupUpdate(GOM8010Group_TypeDef *group);

/* 供上层在HAL_UART_RxCpltCallback中调用(反馈帧改用定长DMA接收,size固定传
   GO_M8010_FEEDBACK_FRAME_SIZE):按huart实例匹配组内总线上当前等待应答的电机并解析,
   然后释放总线供下一个电机的请求使用。若huart不是本组的总线,或总线上并没有电机在等待应答
   (意外/迟到的事件),直接忽略 */
void GOM8010GroupRxEvent(GOM8010Group_TypeDef *group, UART_HandleTypeDef *huart, uint16_t size);

/* 供上层在HAL_UART_ErrorCallback中调用:反馈帧改用DMA接收后,溢出/帧/噪声等错误会被HAL当作
   阻塞错误处理,自动中止当前DMA接收并将RxState还原为READY,此处按huart实例匹配组内总线上
   当前等待应答的电机并提前释放总线,避免其余电机多等一个GO_M8010_BUS_TIMEOUT_MS才能轮到。
   若huart不是本组的总线,或总线上并没有电机在等待应答,直接忽略 */
void GOM8010GroupRxErrorEvent(GOM8010Group_TypeDef *group, UART_HandleTypeDef *huart);

#endif /* __GO_M8010_H__ */
