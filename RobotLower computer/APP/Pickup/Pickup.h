#ifndef __PICKUP_H
#define __PICKUP_H

#include "RoboticArm.h"

/* 抓取路径参数，单位为 m；联调时按实际机械结构标定。 */
/* 机械臂初始位置，所有抓取路径均相对此位置定义。 */
#define pickup_start_x              (0.0f)
#define pickup_start_z              (0.0f)
#define PICKUP_SAFE_HEIGHT          (0.100f) /* 平移前抬升的安全高度 */
#define PICKUP_TARGET_X             (0.0f) /* KFS 抓取位置 x */
#define PICKUP_TARGET_Z_LOW         (0.0f) /* 低位 KFS 抓取位置 z */
#define PICKUP_TARGET_Z_HIGH        (0.0f) /* 高位 KFS 抓取位置 z */
#define PICKUP_STORAGE_X            (0.0f) /* KFS 存放区域 x */
#define PICKUP_STORAGE_Z            (0.0f) /* KFS 存放区域 z */
#define PICKUP_POSITION_TOLERANCE_X (0.005f)
#define PICKUP_POSITION_TOLERANCE_Z (0.005f)

typedef enum
{
    PICKUP_STATE_VOID = 0,  /* 空闲，等待主控发出新的抓取命令 */
    PICKUP_STATE_RAISE,     /* 原地抬升至安全高度 */
    PICKUP_STATE_APPROACH,  /* 在安全高度移动至 KFS 上方 */
    PICKUP_STATE_LOWER,     /* 下降至抓取位置 */
    PICKUP_STATE_GRIP,      /* 闭合末端执行器，抓取 KFS */
    PICKUP_STATE_RETRACT,   /* 保持当前高度收回至存放区域 */
    PICKUP_STATE_PLACE,     /* 调整至存放高度 */
    PICKUP_STATE_RELEASE,   /* 存放位置到位后松开 KFS */
    PICKUP_STATE_RESET      /* 机械臂复位，随后进入 VOID */
} PickupState_TypeDef;

/* 主控抓取高度指令：0 为低位 KFS，1 为高位 KFS。 */
#define PICKUP_HEIGHT_LOW           (0U)
#define PICKUP_HEIGHT_HIGH          (1U)

/* 与 Flip 共用的末端执行器弱接口，由硬件模块提供强定义。 */
__weak void RoboticArmGripMotion(void);
__weak void RoboticArmReleaseMotion(void);

/*
 * 主控发出抓取命令时将状态设为 RAISE，再周期调用本函数。
 * pickup_height 为主控高度指令：0 抓取低位 KFS，1 抓取高位 KFS。
 */
void RoboticArmPickupMotion(RoboticArm_TypeDef *arm,
                            PickupState_TypeDef *pickup_state,
                            uint8_t pickup_height);

#endif
