#ifndef __PICKUP_H
#define __PICKUP_H

#include "RoboticArm.h"

/* 抓取路径标定值，单位为 m；由机械臂零位和实际 KFS 尺寸确定。 */
#define pickup_start_x (0.0f)        /* 机械臂复位点 x。 */
#define pickup_start_z (0.0f)        /* 机械臂复位点 z。 */
#define pickup_hold_x (0.0f)         /*上斜坡机械臂位置保持x*/
#define pickup_hold_z (0.0f)         /*上斜坡机械臂位置保持z*/
#define PICKUP_TARGET_X (0.68f)       /* KFS 抓取位置 x。 */
#define PICKUP_TARGET_Z_LOW (0.0f)   /* 低位 KFS 的吸取 z。 */
#define PICKUP_TARGET_Z_HIGH (0.24f)  /* 高位 KFS 的吸取 z。 */
#define PICKUP_STORAGE_X (0.0f)      /* 储存区中心 x。 */
#define PICKUP_STORAGE_Z_LOW (0.0f)  /* 储存区底层 KFS 的放置 z。 */
#define PICKUP_STORAGE_Z_HIGH (0.5f) /* 储存区上层KFS放置z */

#define PICKUP_TARGET_X_BACK (0.35f)
#define PICKUP_TARGET_Z_DOWN (0.15f)
#define PICKUP_TARGET_Z2 (0.13f)
#define PICKUP_TARGET_RESET_X (0.07f)

#define PICKUP_POSITION_TOLERANCE_X (0.01f)
#define PICKUP_POSITION_TOLERANCE_Z (0.01f)

typedef enum
{
    PICKUP_STATE_VOID = 0, /* 空闲；主控置为 RAISE 以启动一次抓取。 */
    PICKUP_STATE_RAISE,    /* 原地上升 */
    PICKUP_STATE_RAISE_WAIT,
    PICKUP_STATE_FORWARD, /*移至KFS*/
    PICKUP_STATE_FORWARD_WAIT,
    PICKUP_STATE_BACKWARD,
    PICKUP_STATE_BACKWARD_WAIT,
    PICKUP_STATE_RAISE2,
    PICKUP_STATE_RAISE2_WAIT,
    PICKUP_STATE_ROTATION,
    PICKUP_STATE_GRIP, /* 根据动作类型选择后续路径。 */
    PICKUP_STATE_STORE_HIGH,
    PICKUP_STATE_STORE_HIGH_WAIT,
    PICKUP_STATE_STORE_HIGH2,
    PICKUP_STATE_STORE_HIGH2_WAIT,
    PICKUP_STATE_STORE_LOW,
    PICKUP_STATE_STORE_LOW_WAIT,
    PICKUP_STATE_STORE_LOW2,
    PICKUP_STATE_STORE_LOW2_WAIT,
    PICKUP_STATE_RELEASE, /* 仅储存动作关闭吸盘，释放 KFS。 */
    PICKUP_STATE_RESET,   /* 放置完成后返回复位点。 */
    PICKUP_STATE_RESET_WAIT,
    PICKUP_STATE_RESET1,
    PICKUP_STATE_RESET2,
    PICKUP_STATE_RESET2_WAIT,
    PICKUP_STATE_HOLD /* 吸盘保持开启，等待主控切换到后续任务。 */
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
 主控在启动一次动作前设置 action、state=RAISE，再置 active=1；complete 的 0/1/2
 分别表示放低位、放高位、hold。动作到达 VOID 后由状态机递增 complete。
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
