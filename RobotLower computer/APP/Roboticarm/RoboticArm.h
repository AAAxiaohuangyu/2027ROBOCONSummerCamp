#ifndef __ROBOTICARM_H_
#define __ROBOTICARM_H_

#include "J60.h"
#include "GO_M8010.h"
#include "Servo.h"

/*
 * 本文件是机械臂的统一上层接口。Flip、Pickup 与手动控制模块只应在这里
 * 设置末端目标或读取末端状态，不应直接修改 J60、GO 电机的控制结构体。
 *
 * 本层负责坐标与电机角度之间的换算、末端状态更新以及到位判断；CAN 帧、
 * RS485 帧和气泵 GPIO 电平等硬件细节分别留在 J60、GO_M8010、GasPump 模块。
 */

/*
机械臂末端坐标模型(底盘平面几何中心为坐标原点,单位:m/rad):

1. 升降机构(J60驱动),安装于底盘平面上的固定点(ROBOTICARM_BASE_X, ROBOTICARM_BASE_Y, 0),
   沿z轴竖直移动,电机转角theta_lift与高度的关系:
       height = ROBOTICARM_LIFT_K * theta_lift + ROBOTICARM_LIFT_THRESHOLD
   该点始终在(ROBOTICARM_BASE_X, ROBOTICARM_BASE_Y, height)。

2. 前后平移机构(GO电机驱动),安装于升降机构顶端,原理与升降机构类似,电机转角theta_forward
   与前伸距离的关系:
       distance = ROBOTICARM_FORWARD_K * theta_forward + ROBOTICARM_FORWARD_THRESHOLD
   该点位于(ROBOTICARM_BASE_X + distance, ROBOTICARM_BASE_Y, height)。

3. 该点固连一根长度为ROBOTICARM_ROD_LENGTH、沿y轴方向的刚性杆;机械结构上杆末端(执行末端)
   在z方向相对该点始终高出ROBOTICARM_END_Z_OFFSET,故末端坐标:
       end_x = ROBOTICARM_BASE_X + distance
       end_y = ROBOTICARM_BASE_Y + ROBOTICARM_ROD_LENGTH
       end_z = height + ROBOTICARM_END_Z_OFFSET
   可见end_y由安装位置和杆长唯一确定,不受电机控制;可控自由度为end_x、end_z。

4. 另有一个GO电机驱动该杆绕自身轴线自转(如末端夹持器的roll),该电机转角与杆自转角度
   rod_rotation直接相等,不经过任何线性换算,且自转不改变end_x/end_y/end_z。

以上ROBOTICARM_BASE_X、ROBOTICARM_BASE_Y、ROBOTICARM_LIFT_K、ROBOTICARM_LIFT_THRESHOLD、
ROBOTICARM_FORWARD_K、ROBOTICARM_FORWARD_THRESHOLD、ROBOTICARM_ROD_LENGTH、ROBOTICARM_END_Z_OFFSET
均为待实测标定的安装、机械参数,当前为占位值,标定后请在此处直接改数值。
*/

#define ROBOTICARM_BASE_X (0.0f) /* 升降机构安装点在底盘坐标系下的x坐标,待标定 */
#define ROBOTICARM_BASE_Y (0.0f) /* 升降机构安装点在底盘坐标系下的y坐标,待标定 */

#define ROBOTICARM_LIFT_K (0.025f)     /* 升降机构:height = LIFT_K * theta + LIFT_THRESHOLD,待标定 */
#define ROBOTICARM_LIFT_THRESHOLD (0.0f) /* 升降机构:theta=0时对应的初始高度,待标定 */

#define ROBOTICARM_FORWARD_K (0.030f)     /* 前后机构:distance = FORWARD_K * theta + FORWARD_THRESHOLD,待标定 */
#define ROBOTICARM_FORWARD_THRESHOLD (0.0f) /* 前后机构:theta=0时对应的初始前伸距离,待标定 */

#define ROBOTICARM_ROD_LENGTH (0.0f) /* 末端固定杆长度,沿y轴方向,待标定 */

#define ROBOTICARM_END_Z_OFFSET (0.084f) /* 杆末端相对(BASE_X, BASE_Y, height)在z方向的固定高出量,已实测84mm */

#define ROBOTICARM_CONTROL_PERIOD_MS (3U)

#define LOAD_MASS (2.512f) /*负载质量,kg*/
#define STRATEGYALGORITHM_GRAVITY_ACCEL (9.8f) /* 重力加速度,m/s^2 */
#define GravityCompensationLift (LOAD_MASS * STRATEGYALGORITHM_GRAVITY_ACCEL * ROBOTICARM_LIFT_K)

/* GO 电机组只保留前后轴；原 ID 7 自转轴已替换为 JP6 舵机。 */
enum
{
   ROBOTICARM_GO_FORWARD = 0,
};

typedef struct
{
   J60Motor_TypeDef lift_motor;    /* 升降机构电机,控制高度end_z */
   GOM8010Group_TypeDef go_motors; /* GO 电机组：仅前后轴 ID 3。 */
   Servo_TypeDef rotate_servo;     /* 自转轴舵机：JP6/PB5/TIM3_CH2。 */
   uint32_t rotate_command_tick;   /* 最近一次舵机目标下发时间。 */
   uint8_t rotate_motion_active;   /* 1=等待舵机完成相对转动。 */

   float end_x;        /* 末端点x坐标(底盘坐标系),由前后机构位置换算得到 */
   float end_y;        /* 末端点y坐标(底盘坐标系),由安装位置与杆长固定,不受电机控制 */
   float end_z;        /* 末端点z坐标(底盘坐标系),即升降高度height */
   float rod_rotation; /* 杆自转目标角度，单位 rad；舵机无反馈，非真实测量值。 */
} RoboticArm_TypeDef;

/*
 * 数据流：上层动作设置目标 -> 本模块换算为电机目标 -> 独立控制任务周期发送
 * CAN/RS485 帧 -> 中断回调解析反馈 -> 本模块更新 end_x/end_y/end_z/rod_rotation
 * -> 上层使用 PositionReached/RotationReached 决定是否进入下一动作。
 */

/* 初始化机械臂:依次初始化升降(J60/FDCAN)、前后(GO/RS485)、自转(GO/RS485)三个电机驱动,并按
   当前反馈位置(初始为0)计算一次末端坐标与rod_rotation;本函数不下发使能,升降机构使用前需
   额外调用RoboticArmEnable */
void RoboticArmInit(RoboticArm_TypeDef *arm,
                    FDCAN_HandleTypeDef *lift_FDCAN_Handle, uint8_t lift_id,
                    UART_HandleTypeDef *forward_huart, uint8_t forward_id);

/* 下发末端目标坐标(end_x, end_z),内部按几何关系反解为两个电机的目标转角并下发;
   end_y由安装位置与杆长固定,不可控,故不作为参数;lift_torque_feedforward为升降机构(J60)
   电机的前馈力矩,叠加到其kp/kd跟踪输出上,用于重力补偿等场景 */
void RoboticArmSetEndPosition(RoboticArm_TypeDef *arm, float end_x_target, float end_z_target,
                              float lift_torque_feedforward);

/* 下发杆自转目标角度,直接作为自转电机的目标转角下发(两者相等,无需换算) */
void RoboticArmSetRodRotation(RoboticArm_TypeDef *arm, float rotation_target);

/* 周期调用:推进三个电机的规划并发送控制帧,同时按最新反馈更新arm->end_x/end_y/end_z/rod_rotation;
   调用前需已通过J60MotorParseFeedback/GOM8010GroupRxEvent更新对应电机反馈 */
void RoboticArmUpdate(RoboticArm_TypeDef *arm);

/* 使能/失能升降机构(J60);前后、自转机构(GO电机)无需单独使能 */
void RoboticArmEnable(RoboticArm_TypeDef *arm);
void RoboticArmDisable(RoboticArm_TypeDef *arm);

uint8_t PositionReached(RoboticArm_TypeDef *arm,
                        float target_x,
                        float target_z, float x_position_tolerance, float z_position_tolrance);
uint8_t RotationReached(RoboticArm_TypeDef *arm,
                        float target_rotation, float rotation_tolerance);

#endif
