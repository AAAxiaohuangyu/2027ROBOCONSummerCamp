#include "Pickup.h"

typedef enum
{
    PICKUP_OPERATION_STORE_LOW = 0, /* 放至储存区底层。 */
    PICKUP_OPERATION_STORE_HIGH,    /* 放至储存区上层。 */
    PICKUP_OPERATION_STORE_TOP      /* 放至储存区顶层。 */
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
    return PICKUP_STORAGE_Z + (float)operation * PICKUP_STORAGE_STACK_HEIGHT;
}

static void PickupRun(RoboticArm_TypeDef *arm,
                      PickupState_TypeDef *pickup_state,
                      uint8_t pickup_height,
                      PickupOperation_TypeDef operation,
                      float *flip_target)
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
        /* 先原地升到当前 KFS 的中心高度。 */
        target_x = pickup_start_x;
        target_z = PickupGetTargetZ(pickup_height);
        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);
        if (PositionReached(arm, target_x, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_APPROACH;
        break;

    case PICKUP_STATE_APPROACH:
        /* 保持 KFS 中心高度前伸到吸附位置。 */
        target_x = PICKUP_TARGET_X;
        target_z = PickupGetTargetZ(pickup_height);
        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);
        if (PositionReached(arm, target_x, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_GRIP;
        break;

    case PICKUP_STATE_GRIP:
        /* 吸住 KFS 后记录相对当前姿态的单次 180 度翻转目标。 */
        RoboticArmGripMotion();
        *flip_target = arm->rod_rotation + PICKUP_FLIP_ANGLE;
        *pickup_state = PICKUP_STATE_ROTATE;
        break;

    case PICKUP_STATE_ROTATE:
        RoboticArmSetRodRotation(arm, *flip_target);
        if (RotationReached(arm, *flip_target, PICKUP_FLIP_ANGLE / 90.0f))
            *pickup_state = PICKUP_STATE_RETRACT;
        break;

    case PICKUP_STATE_RETRACT:
        /* 保持抓取高度移动到存放区域上方。 */
        RoboticArmSetEndPosition(arm, PICKUP_STORAGE_X, target_z, GravityCompensationLift);
        if (PositionReached(arm, PICKUP_STORAGE_X, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_PLACE;
        break;

    case PICKUP_STATE_PLACE:
        /* 第 1、2 件分别落在堆叠底层和上层。 */
        target_z = PickupGetStorageZ(operation);
        RoboticArmSetEndPosition(arm, PICKUP_STORAGE_X, target_z, GravityCompensationLift);
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
        RoboticArmSetEndPosition(arm, pickup_start_x, pickup_start_z, GravityCompensationLift);
        if (PositionReached(arm, pickup_start_x, pickup_start_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_VOID;
        break;

    default:
        /* 非法状态恢复为空闲状态。 */
        *pickup_state = PICKUP_STATE_VOID;
        break;
    }
}

void RoboticArmPickupStoreLowMotion(RoboticArm_TypeDef *arm,
                                    PickupState_TypeDef *pickup_state,
                                    uint8_t pickup_height, float *flip_target)
{
    /* 主控选择的第一种存放动作：使用储存区底层 z。 */
    PickupRun(arm, pickup_state, pickup_height, PICKUP_OPERATION_STORE_LOW, flip_target);
}

void RoboticArmPickupStoreHighMotion(RoboticArm_TypeDef *arm,
                                     PickupState_TypeDef *pickup_state,
                                     uint8_t pickup_height, float *flip_target)
{
    /* 主控选择的第二种存放动作：使用底层 z 加堆叠高度。 */
    PickupRun(arm, pickup_state, pickup_height, PICKUP_OPERATION_STORE_HIGH, flip_target);
}

void RoboticArmPickupStoreTopMotion(RoboticArm_TypeDef *arm,
                                    PickupState_TypeDef *pickup_state,
                                    uint8_t pickup_height, float *flip_target)
{
    /* 主控选择的第三种存放动作：使用底层 z 加两层堆叠高度。 */
    PickupRun(arm, pickup_state, pickup_height, PICKUP_OPERATION_STORE_TOP, flip_target);
}
