/**
 * Common/bsp_config.h
 * 全局共享外设的语义化宏定义。各模块自己私有的引脚/外设宏放在各自模块文件夹下，
 * 这里只放跨模块共用的资源（通信总线、调试口），避免多人同时改动同一文件。
 */
#ifndef __BSP_CONFIG_H__
#define __BSP_CONFIG_H__

#include "fdcan.h"
#include "usart.h"

/* FDCAN 总线 1：通信总线，四个模块共用 */
#define CANBUS1_HANDLE &hfdcan1

/* USART 1：调试打印口 */
#define DEBUG_UART_HANDLE &huart1
#define DEBUG_UART_INSTANCE USART1

#endif /* __BSP_CONFIG_H__ */
