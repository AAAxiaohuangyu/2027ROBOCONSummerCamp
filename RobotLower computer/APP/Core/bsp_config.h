/**
 * Core/bsp_config.h
 * 各模块挂载的外设句柄（FDCAN/UART等）与节点地址统一集中在此处定义，模块自身
 * 头文件只保留标定参数、状态机等非外设配置，避免多人同时改动各自模块文件夹下
 * 的外设宏而互相冲突。硬件分配变化时只改这里的宏值，不用改各模块的解析代码。
 */
#ifndef __BSP_CONFIG_H__
#define __BSP_CONFIG_H__

#include "fdcan.h"
#include "usart.h"
#include "tim.h"

/* 数学常量：float精度的圆周率，统一来源，避免各模块各自重复定义、精度不一致 */
#define BSP_PI (3.14159265358979323846f)

/* 编码器(Encoder)双万向轮挂载外设句柄与节点ID，供EncoderInit()调用者传入。
   x_axis(ID1)、y_axis(ID2)目前共用同一路FDCAN，靠节点id区分；更换CAN接口或
   编码器ID时只改这里的宏值 */
#define ENCODER_X_FDCAN_HANDLE (&hfdcan3)
#define ENCODER_X_NODE_ID      1u
#define ENCODER_Y_FDCAN_HANDLE (&hfdcan3)
#define ENCODER_Y_NODE_ID      2u

#define YIS512_FDCAN_HANDLE (&hfdcan3)

/* 机械臂(RoboticArm)三电机挂载外设句柄与地址,占位:CubeMX尚未分配对应FDCAN/RS485外设、
   地址也未实测确定,暂不接实际句柄,待确定后直接改这里的宏值即可 */
#define ROBOTICARM_LIFT_FDCAN_HANDLE &hfdcan1 /* 升降电机(J60)FDCAN句柄,占位待定 */
#define ROBOTICARM_LIFT_ID 1u                /* 升降电机(J60)CAN地址,占位待定 */

#define ROBOTICARM_FORWARD_UART_HANDLE &huart3 /* 前后平移电机(GO)RS485串口句柄,占位待定 */
#define ROBOTICARM_FORWARD_ID 3u            /* 前后平移电机(GO)RS485地址,占位待定 */

/* 自转机构改为舵机(PWM开环),复用下方SERVO_PWM_*已初始化的Robot.servo,不再单独占用RS485地址 */

/* 底盘(Chassis)四台M3508电调组挂载外设句柄与控制帧ID,占位:CubeMX尚未分配对应FDCAN外设、
   控制帧ID(M3508_CTRL_ID_1TO4/M3508_CTRL_ID_5TO8)也未确定,暂不接实际句柄,待确定后直接
   改这里的宏值即可 */
#define CHASSIS_FDCAN_HANDLE &hfdcan1 /* 底盘四台M3508电调组FDCAN句柄,占位待定 */
#define CHASSIS_CTRL_ID 0x200u    /* 底盘电调组控制帧ID,占位待定 */

/* 视觉(Vision)模块挂载串口句柄,占位:CubeMX尚未分配对应UART外设,暂不接实际句柄,
   待确定后直接改这里的宏值即可 */
#define VISION_UART_HANDLE (&huart5)

#define ZIGBEE_UART_HANDLE (huart9)

/*JP6 舵机接口*/
#define SERVO_PWM_TIMER_HANDLE &htim2
#define SERVO_PWM_CHANNEL TIM_CHANNEL_2

#endif /* __BSP_CONFIG_H__ */
