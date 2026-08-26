#ifndef __J60_H__
#define __J60_H__

#include "fdcan_common.h"
#include "StrategyAlogrithm.h"
#include <stdint.h>

/*
J60关节电机CAN通信协议(标准帧,DLC 8):

命令帧ID编码(主控->电机): bit[3:0]=电机CAN地址id, bit[4]=response(请求恒为0),
                          bit[7:5]=command(1=失能 2=使能 4=控制),即J60_CommandId()
反馈帧ID编码(电机->主控): 同上编码但response=1,仅在收到一帧控制命令后以该id广播一帧反馈

控制帧(主控->电机,DLC 8,64bit小端紧凑打包,发送目标位置/速度/kp/kd/扭矩前馈):
    bit[0:15]  位置,线性映射至[J60_POSITION_MIN, J60_POSITION_MAX]
    bit[16:29] 速度,线性映射至[J60_VELOCITY_MIN, J60_VELOCITY_MAX]
    bit[30:39] kp,线性映射至[J60_KP_MIN, J60_KP_MAX]
    bit[40:47] kd,线性映射至[J60_KD_MIN, J60_KD_MAX]
    bit[48:63] 扭矩前馈,线性映射至[J60_TORQUE_MIN, J60_TORQUE_MAX]

反馈帧(电机->主控,DLC 8):
    bit[0:19]  位置,线性映射同上(20bit)
    bit[20:39] 速度,线性映射同上(20bit)
    bit[40:55] 扭矩,线性映射同上(16bit)
    bit[56]    温度来源:0=电调,1=电机
    bit[57:63] 温度原始值(7bit),线性映射至[J60_TEMPERATURE_OFFSET, J60_TEMPERATURE_OFFSET+J60_TEMPERATURE_SCALE]
*/

#define J60_POSITION_MIN (-40.0f)
#define J60_POSITION_MAX (40.0f)
#define J60_VELOCITY_MIN (-40.0f)
#define J60_VELOCITY_MAX (40.0f)
#define J60_TORQUE_MIN (-40.0f)
#define J60_TORQUE_MAX (40.0f)
#define J60_KP_MIN (0.0f)
#define J60_KP_MAX (1023.0f)
#define J60_KD_MIN (0.0f)
#define J60_KD_MAX (51.0f)

#define J60_ID_MIN 0u /* 电机CAN地址占4bit */
#define J60_ID_MAX 15u

#define J60_CAN_FRAME_LENGTH 8U
#define J60_RESPONSE_REQUEST 0U
#define J60_RESPONSE_FEEDBACK 1U
#define J60_CAN_ID_RESPONSE_SHIFT 4U
#define J60_CAN_ID_COMMAND_SHIFT 5U
#define J60_CMD_DISABLE 1U
#define J60_CMD_ENABLE 2U
#define J60_CMD_CONTROL 4U

#define J60_CONTROL_POSITION_BITS 16U
#define J60_CONTROL_VELOCITY_BITS 14U
#define J60_CONTROL_KP_BITS 10U
#define J60_CONTROL_KD_BITS 8U
#define J60_CONTROL_TORQUE_BITS 16U
#define J60_CONTROL_VELOCITY_SHIFT 16U
#define J60_CONTROL_KP_SHIFT 30U
#define J60_CONTROL_KD_SHIFT 40U
#define J60_CONTROL_TORQUE_SHIFT 48U

#define J60_FEEDBACK_POSITION_BITS 20U
#define J60_FEEDBACK_VELOCITY_BITS 20U
#define J60_FEEDBACK_TORQUE_BITS 16U
#define J60_FEEDBACK_POSITION_MASK 0xFFFFFU
#define J60_FEEDBACK_VELOCITY_SHIFT 20U
#define J60_FEEDBACK_TORQUE_SHIFT 40U
#define J60_FEEDBACK_TORQUE_MASK 0xFFFFU
#define J60_FEEDBACK_TEMP_SENSOR_SHIFT 56U
#define J60_FEEDBACK_TEMP_SENSOR_MASK 1U
#define J60_FEEDBACK_TEMP_SHIFT 57U
#define J60_FEEDBACK_TEMP_MASK 0x7FU
#define J60_TEMPERATURE_SCALE 220.0f
#define J60_TEMPERATURE_RAW_MAX 127.0f
#define J60_TEMPERATURE_OFFSET (-20.0f)

/* 位置控制内置速度规划(七段S曲线)默认参数,调参从这里改 */
#define J60_POS_CTRL_A_MAX 60.0f /* 加速度上限 */
#define J60_POS_CTRL_V_MAX 40.0f /* 速度上限 */
#define J60_POS_CTRL_J 60.0f    /* 加加速度(jerk)上限 */
#define J60_POS_CTRL_KP 12.0f   /* 位置环增益kp */
#define J60_POS_CTRL_KD 2.5f    /* 速度环增益kd */

/* 定速模式默认参数:不做位置跟踪(kp=0),速度环增益沿用位置模式的kd */
#define J60_VEL_CTRL_KP 0.0f
#define J60_VEL_CTRL_KD 3.5f

typedef enum
{
    J60_CTRL_MODE_POSITION = 0, /* 原控制模式:S速度规划闭环目标位置 */
    J60_CTRL_MODE_VELOCITY = 1, /* 定速模式:直接跟踪目标速度,kp=0 */
} J60CtrlMode_TypeDef;

typedef struct
{
    float a_max;
    float v_max;
    float j;
    float kp;
    float kd;
} J60PositionCtrlParam_TypeDef;

typedef struct
{
    float kp;
    float kd;
} J60VelocityCtrlParam_TypeDef;

/* 力位速混合控制的控制侧:内置速度规划(SpeedPlan),把"目标位置"闭环为torque前馈为0、kp/kd跟踪的
   position/speed指令,周期打包为控制帧发送 */
typedef struct
{
    float position;
    float velocity;
    float torque;
    float kp;
    float kd;
    SpeedPlan_TypeDef plan;
    float position_target;    /* 目标位置,模式0使用 */
    float velocity_target;    /* 目标速度,模式1使用 */
    float torque_feedforward; /* 前馈扭矩,由上层指定,叠加到kp/kd跟踪输出上 */
    J60CtrlMode_TypeDef mode;
    J60PositionCtrlParam_TypeDef position_param;
    J60VelocityCtrlParam_TypeDef velocity_param;
} J60Control_TypeDef;

typedef struct
{
    float position; /* 已减去开机后第一帧位置(position_offset),即以上电时的位置为软件零点 */
    float velocity;
    float torque;
    float temperature;
    uint8_t temperature_is_motor;

    float position_offset;  /* 开机后第一帧的原始位置,作为软件零点基准,由update_cnt==0时捕获 */
    uint32_t update_cnt; // 累计收到反馈帧的次数,可用于判断电机是否离线
} J60Feedback_TypeDef;

typedef struct
{
    uint8_t id;                        /* 电机CAN地址,0~15 */
    FDCAN_HandleTypeDef *FDCAN_Handle; /* 该电机挂载的FDCAN总线实例,支持同一份驱动挂多路总线 */
    J60Control_TypeDef control;
    J60Feedback_TypeDef feedback;
} J60Motor_TypeDef;

/* 初始化电机:control(含plan、按J60_POS_CTRL_*默认参数)、feedback均在此一并初始化;
   本函数不配置FDCAN过滤器,总线过滤器范围需由调用者通过FDCANStandardInit统一配置以覆盖所有J60反馈id */
void J60MotorInit(J60Motor_TypeDef *motor, FDCAN_HandleTypeDef *FDCAN_Handle, uint8_t id);

/* 下发新的目标位置,切换为模式0(位置模式)并(重新)触发规划;运动中调用即为打断 */
void J60MotorSetTarget(J60Motor_TypeDef *motor, float position_target);

/* 下发新的目标速度,切换为模式1(定速模式):不做位置规划,kp=0,仅由kd跟踪目标速度 */
void J60MotorSetVelocityTarget(J60Motor_TypeDef *motor, float velocity_target);

/* 下发前馈扭矩,叠加到kp/kd跟踪规划轨迹的输出上;默认0 */
void J60MotorSetTorqueFeedforward(J60Motor_TypeDef *motor, float torque_feedforward);

/* 下发使能/失能命令帧(不含数据) */
void J60MotorEnable(J60Motor_TypeDef *motor);
void J60MotorDisable(J60Motor_TypeDef *motor);

/* 解析一帧反馈数据到该电机:先由std_id判断该帧是否为本电机的反馈帧,命中则解析反馈并返回1,
   不命中则不作任何处理并返回0;调用者在HAL_FDCAN_RxFifo0Callback中取得std_id和数据后调用,
   可依次对多台电机尝试直至命中 */
uint8_t J60MotorParseFeedback(J60Motor_TypeDef *motor, uint32_t std_id, const uint8_t *rx_data);

/* 周期调用:推进规划并发送一次控制帧;调用前需已通过J60MotorParseFeedback更新motor->feedback */
void J60MotorUpdate(J60Motor_TypeDef *motor);

#endif
