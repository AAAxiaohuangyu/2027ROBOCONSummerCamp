#include "Pickup.h"

typedef enum
{
    PICKUP_OPERATION_STORE_LOW = 0, /* 放至储存区底层。 */
    PICKUP_OPERATION_STORE_HIGH,    /* 放至储存区上层。 */
    PICKUP_OPERATION_HOLD           /* 只吸附，不进入储存区。 */
} PickupOperation_TypeDef;

static float PickupGetTargetZ(uint8_t pickup_height)
{
    if (pickup_height == PICKUP_HEIGHT_HIGH)
    {
        return PICKUP_TARGET_Z_HIGH;
    }

    return PICKUP_TARGET_Z_LOW;
}

static float PickupGetStorageZ(PickupOperation_TypeDef operation)
{
    if (operation == PICKUP_OPERATION_STORE_HIGH)
    {
        return PICKUP_STORAGE_Z + PICKUP_STORAGE_STACK_HEIGHT;
    }

    return PICKUP_STORAGE_Z;
}

static void PickupRun(RoboticArm_TypeDef *arm,
                      PickupState_TypeDef *pickup_state,
                      uint8_t pickup_height,
                      PickupOperation_TypeDef operation)
{
    float target_x;
    float target_z;

    target_z = PickupGetTargetZ(pickup_height);

    switch (*pickup_state)
    {
    case PICKUP_STATE_VOID:
        /* 空闲状态不输出动作，等待主控启动下一次抓取。 */
        break;

    case PICKUP_STATE_RAISE:
        /* 先原地抬升，避免平移时碰撞或推动 KFS。 */
        target_x = pickup_start_x;
        target_z = pickup_start_z + PICKUP_SAFE_HEIGHT;
        RoboticArmSetEndPosition(arm, target_x, target_z);
        if (PositionReached(arm, target_x, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_APPROACH;
        break;

    case PICKUP_STATE_APPROACH:
        /* 在安全高度移动至抓取点上方。 */
        target_x = PICKUP_TARGET_X;
        target_z = pickup_start_z + PICKUP_SAFE_HEIGHT;
        RoboticArmSetEndPosition(arm, target_x, target_z);
        if (PositionReached(arm, target_x, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_LOWER;
        break;

    case PICKUP_STATE_LOWER:
        /* 下降到 KFS 的抓取高度。 */
        RoboticArmSetEndPosition(arm, PICKUP_TARGET_X, target_z);
        if (PositionReached(arm, PICKUP_TARGET_X, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_GRIP;
        break;

    case PICKUP_STATE_GRIP:
        /* 第 3 件不进入储存区，保持真空；前两件分别存入底层和高层。 */
        RoboticArmGripMotion();
        if (operation == PICKUP_OPERATION_HOLD)
        {
            /* 第三个 KFS 停在抓取点，吸盘持续开启。 */
            *pickup_state = PICKUP_STATE_HOLD;
        }
        else
        {
            *pickup_state = PICKUP_STATE_RETRACT;
        }
        break;

    case PICKUP_STATE_RETRACT:
        /* 保持抓取高度移动到存放区域上方。 */
        RoboticArmSetEndPosition(arm, PICKUP_STORAGE_X, target_z);
        if (PositionReached(arm, PICKUP_STORAGE_X, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_PLACE;
        break;

    case PICKUP_STATE_PLACE:
        /* 第 1、2 件分别落在堆叠底层和上层。 */
        target_z = PickupGetStorageZ(operation);
        RoboticArmSetEndPosition(arm, PICKUP_STORAGE_X, target_z);
        if (PositionReached(arm, PICKUP_STORAGE_X, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_RELEASE;
        break;

    case PICKUP_STATE_RELEASE:
        /* 仅在存放位置确认到位后松开 KFS。 */
        RoboticArmReleaseMotion();
        *pickup_state = PICKUP_STATE_RESET;
        break;

    case PICKUP_STATE_RESET:
        /* 复位完成后进入空闲状态，等待下一次抓取命令。 */
        RoboticArmSetEndPosition(arm, pickup_start_x, pickup_start_z);
        if (PositionReached(arm, pickup_start_x, pickup_start_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_VOID;
        break;

    case PICKUP_STATE_HOLD:
        /* 保持吸盘开启；由后续任务显式释放第三个 KFS。 */
        break;

    default:
        /* 非法状态恢复为空闲状态。 */
        *pickup_state = PICKUP_STATE_VOID;
        break;
    }
}

void RoboticArmPickupStoreLowMotion(RoboticArm_TypeDef *arm,
                                    PickupState_TypeDef *pickup_state,
                                    uint8_t pickup_height)
{
    /* 主控选择的第一种存放动作：使用储存区底层 z。 */
    PickupRun(arm, pickup_state, pickup_height, PICKUP_OPERATION_STORE_LOW);
}

void RoboticArmPickupStoreHighMotion(RoboticArm_TypeDef *arm,
                                     PickupState_TypeDef *pickup_state,
                                     uint8_t pickup_height)
{
    /* 主控选择的第二种存放动作：使用底层 z 加堆叠高度。 */
    PickupRun(arm, pickup_state, pickup_height, PICKUP_OPERATION_STORE_HIGH);
}

void RoboticArmPickupHoldMotion(RoboticArm_TypeDef *arm,
                                PickupState_TypeDef *pickup_state,
                                uint8_t pickup_height)
{
    /* 主控选择的第三种动作：吸附后保持真空，不执行放置或复位。 */
    PickupRun(arm, pickup_state, pickup_height, PICKUP_OPERATION_HOLD);
}
