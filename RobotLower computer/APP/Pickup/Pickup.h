#ifndef __PICKUP_H
#define __PICKUP_H

#include "RoboticArm.h"

/* 抓取路径标定值，单位为 m；由机械臂零位和实际 KFS 尺寸确定。 */
#define pickup_start_x (0.0f)        /* 机械臂复位点 x。 */
#define pickup_start_z (0.0f)        /* 机械臂复位点 z。 */
#define pickup_hold_x (0.0f)         /*上斜坡机械臂位置保持x*/
#define pickup_hold_z (0.0f)         /*上斜坡机械臂位置保持z*/
#define PICKUP_TARGET_X (0.55f)       /* KFS 抓取位置 x。 */
#define PICKUP_TARGET_Z_LOW (0.0f)   /* 低位 KFS 的吸取 z。 */
#define PICKUP_TARGET_Z_HIGH (0.27f)  /* 高位 KFS 的吸取 z。 */
#define PICKUP_STORAGE_X (0.0f)      /* 储存区中心 x。 */
#define PICKUP_STORAGE_Z_LOW (0.0f)  /* 储存区底层 KFS 的放置 z。 */
#define PICKUP_STORAGE_Z_HIGH (0.0f) /* 储存区上层KFS放置z */

#define PICKUP_POSITION_TOLERANCE_X (0.01f)
#define PICKUP_POSITION_TOLERANCE_Z (0.01f)

#define PICKUP_POSITION_TOLERANCE_X_ALT (0.005f)
#define PICKUP_POSITION_TOLERANCE_Z_ALT (0.005f)

typedef enum
{
    PICKUP_STATE_VOID = 0, /* 空闲；主控置为 RAISE 以启动一次抓取。 */
    PICKUP_STATE_RAISE,    /* 原地上升 */
    PICKUP_STATE_FORWARD,  /*移至KFS*/
    PICKUP_STATE_DOWN,
    PICKUP_STATE_GRIP,    /* 开启吸盘并根据动作类型选择后续路径。 */
    PICKUP_STATE_RETRACT, /* 保持抓取高度平移至储存区上方。 */
    PICKUP_STATE_PLACE,   /* 降至低位或高位储存坐标。 */
    PICKUP_STATE_RELEASE, /* 仅储存动作关闭吸盘，释放 KFS。 */
    PICKUP_STATE_RESET,   /* 放置完成后返回复位点。 */
    PICKUP_STATE_HOLD     /* 吸盘保持开启，等待主控切换到后续任务。 */
} PickupState_TypeDef;

typedef enum
{
    PICKUP_TASK_ACTION_NONE = 0,
    PICKUP_TASK_ACTION_STORE_LOW_LOW,
    PICKUP_TASK_ACTION_STORE_LOW_HIGH,
    PICKUP_TASK_ACTION_STORE_HIGH_LOW,
    PICKUP_TASK_ACTION_STORE_HIGH_HIGH,
    PICKUP_TASK_ACTION_HOLD_LOW,
    PICKUP_TASK_ACTION_HOLD_HIGH
} PickupTaskAction_TypeDef;

typedef struct
{
    PickupState_TypeDef state;
    PickupTaskAction_TypeDef action;
    uint8_t active;
    uint8_t complete;
} Pickup_TypeDef;

#define PICKUP_HEIGHT_LOW (0U)
#define PICKUP_HEIGHT_HIGH (1U)

/*
 顶层主控在启动一次动作前设置 action、state=RAISE、complete=0，再置 active=1；
 Pickup 任务只在 active 为真时推进。Store 和 Hold 到达 VOID 后由任务置 complete=1；
 Hold 保持真空并移动至保持位置，但不会调用 RoboticArmReleaseMotion。
 */
void RoboticArmPickupStoreLowMotion(RoboticArm_TypeDef *arm,
                                    PickupState_TypeDef *pickup_state,
                                    uint8_t pickup_height, uint8_t *complete);
void RoboticArmPickupStoreHighMotion(RoboticArm_TypeDef *arm,
                                     PickupState_TypeDef *pickup_state,
                                     uint8_t pickup_height, uint8_t *complete);
void RoboticArmPickupHoldMotion(RoboticArm_TypeDef *arm,
                                PickupState_TypeDef *pickup_state,
                                uint8_t pickup_height, uint8_t *complete);

#endif
