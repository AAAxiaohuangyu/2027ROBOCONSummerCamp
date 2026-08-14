#ifndef __PICKUP_H
#define __PICKUP_H

#include "RoboticArm.h"

/* 抓取路径标定值，单位为 m；由机械臂零位和实际 KFS 尺寸确定。 */
#define pickup_start_x              (0.0f) /* 机械臂复位点 x。 */
#define pickup_start_z              (0.0f) /* 机械臂复位点 z。 */
#define PICKUP_SAFE_HEIGHT          (0.100f) /* 所有水平移动前使用的防碰撞高度。 */
#define PICKUP_TARGET_X             (0.0f) /* KFS 抓取位置 x。 */
#define PICKUP_TARGET_Z_LOW         (0.0f) /* 低位 KFS 的吸取 z。 */
#define PICKUP_TARGET_Z_HIGH        (0.0f) /* 高位 KFS 的吸取 z。 */
#define PICKUP_STORAGE_X            (0.0f) /* 储存区中心 x。 */
#define PICKUP_STORAGE_Z            (0.0f) /* 储存区底层 KFS 的放置 z。 */
#define PICKUP_STORAGE_STACK_HEIGHT (0.0f) /* 上层 KFS 相对底层的 z 增量。 */
#define PICKUP_POSITION_TOLERANCE_X (0.005f)
#define PICKUP_POSITION_TOLERANCE_Z (0.005f)

typedef enum
{
    PICKUP_STATE_VOID = 0, /* 空闲；主控置为 RAISE 以启动一次抓取。 */
    PICKUP_STATE_RAISE,    /* 原地升至安全高度。 */
    PICKUP_STATE_APPROACH, /* 在安全高度移动至抓取点正上方。 */
    PICKUP_STATE_LOWER,    /* 下降至当前 KFS 的抓取高度。 */
    PICKUP_STATE_GRIP,     /* 开启吸盘并根据动作类型选择后续路径。 */
    PICKUP_STATE_RETRACT,  /* 保持抓取高度平移至储存区上方。 */
    PICKUP_STATE_PLACE,    /* 降至低位或高位储存坐标。 */
    PICKUP_STATE_RELEASE,  /* 仅储存动作关闭吸盘，释放 KFS。 */
    PICKUP_STATE_RESET,    /* 放置完成后返回复位点。 */
    PICKUP_STATE_HOLD       /* 吸盘保持开启，等待主控切换到后续任务。 */
} PickupState_TypeDef;

#define PICKUP_HEIGHT_LOW           (0U)
#define PICKUP_HEIGHT_HIGH          (1U)

__weak void RoboticArmGripMotion(void);
__weak void RoboticArmReleaseMotion(void);

/*
 * 主控负责 KFS 计数；每次动作开始前将 pickup_state 置为 RAISE，随后周期调用对应入口。
 * StoreLow 和 StoreHigh 都在放置后复位为 VOID；Hold 在吸附后停在 HOLD，
 * 不会调用 RoboticArmReleaseMotion，需由后续任务显式处理该 KFS。
 */
void RoboticArmPickupStoreLowMotion(RoboticArm_TypeDef *arm,
                                    PickupState_TypeDef *pickup_state,
                                    uint8_t pickup_height);
void RoboticArmPickupStoreHighMotion(RoboticArm_TypeDef *arm,
                                     PickupState_TypeDef *pickup_state,
                                     uint8_t pickup_height);
void RoboticArmPickupHoldMotion(RoboticArm_TypeDef *arm,
                                PickupState_TypeDef *pickup_state,
                                uint8_t pickup_height);

#endif
