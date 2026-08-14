/**
 * Core/bsp_config.h
 * 全局共享外设的语义化宏定义。各模块自己私有的引脚/外设宏放在各自模块文件夹下，
 * 这里只放跨模块共用的资源（通信总线、调试口），避免多人同时改动同一文件。
 */
#ifndef __BSP_CONFIG_H__
#define __BSP_CONFIG_H__

#include "fdcan.h"
#include "usart.h"

/* FDCAN 总线 1：通信总线，四个模块共用 */
#define CANBUS1_HANDLE &hfdcan2

/* USART 1：调试打印口 */
#define DEBUG_UART_HANDLE &huart1
#define DEBUG_UART_INSTANCE USART1

/* 机械臂(RoboticArm)三电机挂载外设句柄与地址,占位:CubeMX尚未分配对应FDCAN/RS485外设、
   地址也未实测确定,暂不接实际句柄,待确定后直接改这里的宏值即可 */
#define ROBOTICARM_LIFT_FDCAN_HANDLE &hfdcan2 /* 升降电机(J60)FDCAN句柄,占位待定 */
#define ROBOTICARM_LIFT_ID 1u                /* 升降电机(J60)CAN地址,占位待定 */

#define ROBOTICARM_FORWARD_UART_HANDLE NULL /* 前后平移电机(GO)RS485串口句柄,占位待定 */
#define ROBOTICARM_FORWARD_ID 3u            /* 前后平移电机(GO)RS485地址,占位待定 */

#define ROBOTICARM_ROTATE_UART_HANDLE NULL /* 自转电机(GO)RS485串口句柄,占位待定 */
#define ROBOTICARM_ROTATE_ID 7u            /* 自转电机(GO)RS485地址,占位待定 */

/* 底盘(Chassis)四台M3508电调组挂载外设句柄与控制帧ID,占位:CubeMX尚未分配对应FDCAN外设、
   控制帧ID(M3508_CTRL_ID_1TO4/M3508_CTRL_ID_5TO8)也未确定,暂不接实际句柄,待确定后直接
   改这里的宏值即可 */
#define CHASSIS_FDCAN_HANDLE NULL /* 底盘四台M3508电调组FDCAN句柄,占位待定 */
#define CHASSIS_CTRL_ID 0x200u    /* 底盘电调组控制帧ID,占位待定 */

#endif /* __BSP_CONFIG_H__ */
