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
    /* 三个公开接口复用同一状态机；operation 只决定吸附后的处理方式。 */
    float target_x;
    float target_z;

    target_z = PickupGetTargetZ(pickup_height);

    switch (*pickup_state)
    {
    case PICKUP_STATE_VOID:
        /* 空闲状态不输出动作，等待主控启动下一次抓取。 */
        break;

    case PICKUP_STATE_RAISE:
        /* 吸盘默认朝向 KFS，先原地上升到当前 KFS 的指定抓取高度。 */
        target_x = pickup_start_x;
        target_z = PickupGetTargetZ(pickup_height);
        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);
        if (PositionReached(arm, target_x, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_APPROACH;
        break;

    case PICKUP_STATE_APPROACH:
        /* 保持指定高度向前伸出，直接到达 KFS 抓取位置。 */
        target_x = PICKUP_TARGET_X;
        target_z = PickupGetTargetZ(pickup_height);
        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);
        if (PositionReached(arm, target_x, target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
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
            /* 前两件先翻转 180 度，再返回存放区。 */
            *pickup_state = PICKUP_STATE_ROTATE;
        }
        break;

    case PICKUP_STATE_ROTATE:
        /* 舵机角度使用弧度；180 度参数集中在 Pickup.h。 */
        RoboticArmSetRodRotation(arm, PICKUP_SERVO_FLIP_ANGLE_RAD);
        if (RotationReached(arm, PICKUP_SERVO_FLIP_ANGLE_RAD, PICKUP_SERVO_ANGLE_TOLERANCE_RAD))
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
        *pickup_state = PICKUP_STATE_RESET_ROTATION;
        break;

    case PICKUP_STATE_RESET_ROTATION:
        /* KFS 放下后，舵机回到 0 度，准备下一次抓取。 */
        RoboticArmSetRodRotation(arm, PICKUP_SERVO_HOME_ANGLE_RAD);
        if (RotationReached(arm, PICKUP_SERVO_HOME_ANGLE_RAD, PICKUP_SERVO_ANGLE_TOLERANCE_RAD))
            *pickup_state = PICKUP_STATE_RESET;
        break;

    case PICKUP_STATE_RESET:
        /* 复位完成后进入空闲状态，等待下一次抓取命令。 */
        RoboticArmSetEndPosition(arm, pickup_start_x, pickup_start_z, GravityCompensationLift);
        if (PositionReached(arm, pickup_start_x, pickup_start_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_VOID;
        break;

    case PICKUP_STATE_HOLD:
        /* 保持状态不会自动复位；后续比赛任务应在合适时机显式释放该 KFS。 */
        /* 保持吸盘开启；由后续任务显式释放第三个 KFS。 */
        break;

    默认:
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
