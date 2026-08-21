#ifndef __BSP_CALLBACK_H_
#define __BSP_CALLBACK_H_

/*
 * HAL弱回调函数(HAL_FDCAN_RxFifo0Callback、HAL_FDCAN_RxFifo1Callback、HAL_UARTEx_RxEventCallback)
 * 全局唯一,不能由各电机驱动模块各自实现,统一在本文件中按hfdcan/huart实例分发给对应电机的
 * ParseFeedback,分发命中后按驱动约定重新挂起下一次接收。各电机所属的FDCAN/UART句柄见
 * Core/bsp_config.h。FIFO0目前挂J60升降电机与底盘M3508电调组(过滤器0,hfdcan1),FIFO1挂
 * 编码器(过滤器0,hfdcan3)。
 */

#endif
