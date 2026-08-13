#include "Pickup.h"

static float PickupGetTargetZ(uint8_t pickup_height)
{
    if (pickup_height == PICKUP_HEIGHT_HIGH)
    {
        return PICKUP_TARGET_Z_HIGH;
    }

    return PICKUP_TARGET_Z_LOW;
}

void RoboticArmPickupMotion(RoboticArm_TypeDef *arm,
                            PickupState_TypeDef *pickup_state,
                            uint8_t pickup_height)
{
    float target_x;
    float target_z;

    target_z = PickupGetTargetZ(pickup_height);

    switch (*pickup_state)
    {
    case PICKUP_STATE_VOID:
        /* 空闲状态不输出新的动作，等待主控启动下一次抓取。 */
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
        /* 末端执行器闭合后立即进入收回流程。 */
        RoboticArmGripMotion();
        *pickup_state = PICKUP_STATE_RETRACT;
        break;

    case PICKUP_STATE_RETRACT:
        /* 保持抓取高度移动到存放区域上方。 */
        RoboticArmSetEndPosition(arm, PICKUP_STORAGE_X, target_z);
        if (PositionReached(arm, PICKUP_STORAGE_X, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_PLACE;
        break;

    case PICKUP_STATE_PLACE:
        /* 下降或上升至标定的存放高度。 */
        RoboticArmSetEndPosition(arm, PICKUP_STORAGE_X, PICKUP_STORAGE_Z);
        if (PositionReached(arm, PICKUP_STORAGE_X, PICKUP_STORAGE_Z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
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

    default:
        /* 非法状态恢复为空闲状态。 */
        *pickup_state = PICKUP_STATE_VOID;
        break;
    }
}
