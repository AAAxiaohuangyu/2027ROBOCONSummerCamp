#ifndef __ROBOTICARM_H_
#define __ROBOTICARM_H_

#include "J60.h"
#include "GO_M8010.h"
#include "Servo.h"

/*
机械臂末端坐标模型(底盘平面几何中心为坐标原点,单位:m/rad):

1. 升降机构(J60驱动),沿z轴竖直移动,电机转角theta_lift与高度的关系:
       height = ROBOTICARM_LIFT_K * theta_lift
   该点始终在(0, 0, height)。

2. 前后平移机构(GO电机驱动),安装于升降机构顶端,原理与升降机构类似,电机转角theta_forward
   与前伸距离的关系:
       distance = ROBOTICARM_FORWARD_K * theta_forward
   该点位于(distance, 0, height)。

3. 末端坐标:
       end_x = distance
       end_y = 0
       end_z = height
   可控自由度为end_x、end_z。

4. 另有一个舵机(PWM开环,无位置反馈)驱动该杆绕自身轴线自转(如末端夹持器的roll),舵机
   转角与杆自转角度rod_rotation直接相等,不经过任何线性换算;下发目标角度即视为立即到位。

以上ROBOTICARM_LIFT_K、ROBOTICARM_FORWARD_K为待实测标定的安装、机械参数,当前为占位值,
标定后请在此处直接改数值。
*/

#define ROBOTICARM_LIFT_K (-0.021f)     /* 升降机构:height = LIFT_K * theta,待标定 */

#define ROBOTICARM_FORWARD_K (0.032f)     /* 前后机构:distance = FORWARD_K * theta,待标定 */

#define ROBOTICARM_CONTROL_PERIOD_MS (3U)

#define LOAD_MASS (2.0f) /*负载质量,kg*/
#define STRATEGYALGORITHM_GRAVITY_ACCEL (9.8f) /* 重力加速度,m/s^2 */
#define GravityCompensationLift (LOAD_MASS * STRATEGYALGORITHM_GRAVITY_ACCEL * ROBOTICARM_LIFT_K)

/* go_motors组内电机下标:0=前后平移机构(控制end_x) */
enum
{
   ROBOTICARM_GO_FORWARD = 0,
};

typedef struct
{
   J60Motor_TypeDef lift_motor;    /* 升降机构电机,控制高度end_z */
   GOM8010Group_TypeDef go_motors; /* 前后平移GO电机,下标见ROBOTICARM_GO_* */
   Servo_TypeDef rotate_servo;    /* 杆自转舵机*/

   float end_x;        /* 末端点x坐标(底盘坐标系),由前后机构位置换算得到 */
   float end_y;        /* 末端点y坐标(底盘坐标系),固定为0 */
   float end_z;        /* 末端点z坐标(底盘坐标系),即升降高度height */
   float rod_rotation; /* 杆绕自身轴线的自转角度,与rotate_servo->angle直接相等 */
} RoboticArm_TypeDef;

/* 初始化机械臂:依次初始化升降(J60/FDCAN)、前后(GO/RS485)两个电机驱动,并保存rotate_servo
   指针(该舵机须已由调用者初始化,本函数不对其做Init);再按当前反馈位置(初始为0)计算一次
   末端坐标与rod_rotation;本函数不下发使能,升降机构使用前需额外调用RoboticArmEnable */
void RoboticArmInit(RoboticArm_TypeDef *arm,
                    FDCAN_HandleTypeDef *lift_FDCAN_Handle, uint8_t lift_id,
                    UART_HandleTypeDef *forward_huart, uint8_t forward_id, TIM_HandleTypeDef *htim, uint32_t channel);

/* 下发末端目标坐标(end_x, end_z),内部按几何关系反解为两个电机的目标转角并下发;
   end_y固定为0,不可控,故不作为参数;lift_torque_feedforward为升降机构(J60)
   电机的前馈力矩,叠加到其kp/kd跟踪输出上,用于重力补偿等场景 */
void RoboticArmSetEndPosition(RoboticArm_TypeDef *arm, float end_x_target, float end_z_target,
                              float lift_torque_feedforward);

/* 下发杆自转目标角度,直接写入rotate_servo->angle(两者相等,无需换算);舵机开环无反馈,
   下发即视为立即到位 */
void RoboticArmSetRodRotation(RoboticArm_TypeDef *arm, float rotation_target);

/* 周期调用:推进升降/前后两个电机的规划并发送控制帧,同时按最新反馈更新
   arm->end_x/end_y/end_z,并将rotate_servo->angle同步到arm->rod_rotation;
   调用前需已通过J60MotorParseFeedback/GOM8010GroupRxEvent更新对应电机反馈 */
void RoboticArmUpdate(RoboticArm_TypeDef *arm);

/* 使能/失能升降机构(J60);前后机构(GO电机)、自转机构(舵机)无需单独使能 */
void RoboticArmEnable(RoboticArm_TypeDef *arm);
void RoboticArmDisable(RoboticArm_TypeDef *arm);

uint8_t PositionReached(RoboticArm_TypeDef *arm,
                        float target_x,
                        float target_z, float x_position_tolerance, float z_position_tolrance);
uint8_t RotationReached(RoboticArm_TypeDef *arm,
                        float target_rotation, float rotation_tolerance);

#endif
