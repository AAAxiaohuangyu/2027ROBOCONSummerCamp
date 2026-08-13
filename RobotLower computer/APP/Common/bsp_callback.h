#ifndef __BSP_CALLBACK_H_
#define __BSP_CALLBACK_H_

/*
 * HAL弱回调函数(HAL_FDCAN_RxFifo0Callback、HAL_UARTEx_RxEventCallback)全局唯一,不能由
 * 各电机驱动模块各自实现,统一在本文件中按hfdcan/huart实例分发给对应电机的ParseFeedback,
 * 分发命中后按驱动约定重新挂起下一次接收。各电机所属的FDCAN/UART句柄见Core/bsp_config.h。
 */

#endif
