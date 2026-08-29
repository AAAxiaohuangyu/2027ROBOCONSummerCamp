#include "Pickup.h"
#include "GasPumpCLY.h"
#include "cmsis_os2.h"

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

static void PickupRun(RoboticArm_TypeDef *arm,
                      PickupState_TypeDef *pickup_state,
                      uint8_t pickup_height,
                      PickupOperation_TypeDef operation, uint8_t *complete)
{
    switch (*pickup_state)
    {
    case PICKUP_STATE_VOID:
        /* 空闲状态不输出动作，等待主控启动下一次抓取。 */
        break;

    case PICKUP_STATE_RAISE:
        arm->target_rotation = BSP_PI;

        RoboticArmSetRodRotation(arm, arm->target_rotation);

        arm->target_x = pickup_start_x;
        arm->target_z = PickupGetTargetZ(pickup_height);

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        *pickup_state = PICKUP_STATE_RAISE_WAIT;
        break;

    case PICKUP_STATE_RAISE_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_FORWARD;
        break;

    case PICKUP_STATE_FORWARD:
        arm->target_x = PICKUP_TARGET_X;
        arm->target_z = PickupGetTargetZ(pickup_height);
        GasPumpOn();
        CLY_On();
        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);
        *pickup_state = PICKUP_STATE_FORWARD_WAIT;
        break;

    case PICKUP_STATE_FORWARD_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_BACKWARD;
        break;

    case PICKUP_STATE_BACKWARD:
        arm->target_x = PICKUP_TARGET_X - PICKUP_TARGET_X_BACK;
        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);
        *pickup_state = PICKUP_STATE_BACKWARD_WAIT;
        break;

    case PICKUP_STATE_BACKWARD_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_RAISE2;
        break;

    case PICKUP_STATE_RAISE2:
        arm->target_z += PICKUP_TARGET_Z2;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        *pickup_state = PICKUP_STATE_RAISE2_WAIT;
        break;

    case PICKUP_STATE_RAISE2_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_ROTATION;
        break;

    case PICKUP_STATE_ROTATION:
        arm->target_rotation = 0.8f;

        RoboticArmSetRodRotation(arm, arm->target_rotation);

        osDelay(1000);
        *pickup_state = PICKUP_STATE_GRIP;
        break;

    case PICKUP_STATE_GRIP:
        if (operation == PICKUP_OPERATION_HOLD)
        {
            /* 第三个 KFS 停在抓取点，吸盘持续开启。 */
            *pickup_state = PICKUP_STATE_HOLD;
        }
        else if (operation == PICKUP_OPERATION_STORE_HIGH)
        {
            *pickup_state = PICKUP_STATE_STORE_HIGH;
        }
        else if (operation == PICKUP_OPERATION_STORE_LOW)
        {
            *pickup_state = PICKUP_STATE_STORE_LOW;
        }
        break;

    case PICKUP_STATE_STORE_LOW:
        arm->target_x = pickup_start_x;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        *pickup_state = PICKUP_STATE_STORE_LOW_WAIT;
        break;

    case PICKUP_STATE_STORE_LOW_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_STORE_LOW2;
        break;

    case PICKUP_STATE_STORE_LOW2:
        arm->target_z -= PICKUP_TARGET_Z_DOWN;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        *pickup_state = PICKUP_STATE_STORE_LOW2_WAIT;
        break;

    case PICKUP_STATE_STORE_LOW2_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_RELEASE;
        break;

    case PICKUP_STATE_STORE_HIGH:
        arm->target_x = pickup_start_x;
        arm->target_z = pickup_start_z;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        *pickup_state = PICKUP_STATE_STORE_HIGH_WAIT;
        break;

    case PICKUP_STATE_STORE_HIGH_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
            *pickup_state = PICKUP_STATE_RELEASE;
        break;

    case PICKUP_STATE_RELEASE:
        /* 仅在存放位置确认到位后松开 KFS。 */
        CLY_Off();
        osDelay(1000);
        CLY_On();
        *pickup_state = PICKUP_STATE_VOID;
        break;

    case PICKUP_STATE_RESET:
        /* 复位完成后进入空闲状态，等待下一次抓取命令。 */
        arm->target_x = pickup_start_x;
        arm->target_z = pickup_start_z;
        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);
        if (PositionReached(arm, arm->target_x, arm->target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
        {
            *pickup_state = PICKUP_STATE_VOID;
            *complete++;
        }
        break;

    case PICKUP_STATE_HOLD:
        /* 保持吸盘开启；由后续任务显式释放第三个 KFS。 */
        arm->target_x = pickup_hold_x;
        arm->target_z = pickup_hold_z;
        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);
        if (PositionReached(arm, arm->target_x, arm->target_z, PICKUP_POSITION_TOLERANCE_X, PICKUP_POSITION_TOLERANCE_Z))
        {
            *pickup_state = PICKUP_STATE_VOID;
            *complete++;
        }
        break;

    default:
        /* 非法状态恢复为空闲状态。 */
        *pickup_state = PICKUP_STATE_VOID;
        break;
    }
}

void RoboticArmPickupStoreLowMotion(RoboticArm_TypeDef *arm,
                                    PickupState_TypeDef *pickup_state,
                                    uint8_t pickup_height, uint8_t *complete)
{
    /* 主控选择的第一种存放动作：使用储存区底层 z。 */
    PickupRun(arm, pickup_state, pickup_height, PICKUP_OPERATION_STORE_LOW,complete);
}

void RoboticArmPickupStoreHighMotion(RoboticArm_TypeDef *arm,
                                     PickupState_TypeDef *pickup_state,
                                     uint8_t pickup_height,uint8_t *complete)
{
    /* 主控选择的第二种存放动作：使用底层 z 加堆叠高度。 */
    PickupRun(arm, pickup_state, pickup_height, PICKUP_OPERATION_STORE_HIGH,complete);
}

void RoboticArmPickupHoldMotion(RoboticArm_TypeDef *arm,
                                PickupState_TypeDef *pickup_state,
                                uint8_t pickup_height, uint8_t *complete)
{
    /* 主控选择的第三种动作：吸附后保持真空，不执行放置或复位。 */
    PickupRun(arm, pickup_state, pickup_height, PICKUP_OPERATION_HOLD,complete);
}
