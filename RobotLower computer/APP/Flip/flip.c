#include "flip.h"

void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, FlipState_TypeDef *flip_state)
{
    /*
     * 每个目标均相对翻转起点累加，实物标定时可集中调整起点和分段距离。
     * 函数不等待、不延时：到位前保持当前状态，方便与 FreeRTOS 周期调度协作。
     */
    float target_x;
    float target_z;
    float target_rotation;

    switch (*flip_state)
    {
    case FLIP_STATE_UP:
        target_x = flip_start_x;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_FORWARD;

        break;

    case FLIP_STATE_FORWARD:
        target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_GRIP;

        break;

    case FLIP_STATE_GRIP:
        /* 气泵的 GPIO、有效电平与真实吸附结果由 GasPump 和硬件决定。 */
        RoboticArmGripMotion();
        *flip_state = FLIP_STATE_ROTATE;

        break;

    case FLIP_STATE_ROTATE:
        target_rotation = flip_start_rotation +
                          FLIP_ROTATION_ANGLE_1;

        RoboticArmSetRodRotation(arm, target_rotation);

        if (RotationReached(arm, target_rotation, FLIP_ROTATION_TOLERANCE))
            *flip_state = FLIP_STATE_FORWARD_AFTER_ROTATE;

        break;

    case FLIP_STATE_FORWARD_AFTER_ROTATE:
        target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1 +
                   FLIP_FORWARD_DISTANCE_2;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_RELEASE;

        break;

    case FLIP_STATE_RELEASE:
        /* 当前释放动作仅切换控制信号；若需真空释放等待，应在流程设计中明确加入。 */
        RoboticArmReleaseMotion();
        *flip_state = FLIP_STATE_FORWARD_AFTER_RELEASE;

        break;

    case FLIP_STATE_FORWARD_AFTER_RELEASE:
        target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1 +
                   FLIP_FORWARD_DISTANCE_2 +
                   FLIP_FORWARD_DISTANCE_3;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_UP_AFTER_RELEASE;

        break;

    case FLIP_STATE_UP_AFTER_RELEASE:
        target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1 +
                   FLIP_FORWARD_DISTANCE_2 +
                   FLIP_FORWARD_DISTANCE_3;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1 +
                   FLIP_UP_DISTANCE_2;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_BACK;

        break;

    case FLIP_STATE_BACK:
        target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1 +
                   FLIP_FORWARD_DISTANCE_2 +
                   FLIP_FORWARD_DISTANCE_3 -
                   FLIP_BACKWARD_DISTANCE;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1 + FLIP_UP_DISTANCE_2;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_DOWN;

        break;

    case FLIP_STATE_DOWN:
        target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1 +
                   FLIP_FORWARD_DISTANCE_2 +
                   FLIP_FORWARD_DISTANCE_3 -
                   FLIP_BACKWARD_DISTANCE;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1 +
                   FLIP_UP_DISTANCE_2 -
                   FLIP_DOWN_DISTANCE;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_ROTATE_BACK;

        break;

    case FLIP_STATE_ROTATE_BACK:
        /* 舵机的回程目标是初始 0 度；不能再按 GO 电机角度累加两个 π。 */
        target_rotation = flip_start_rotation;

        RoboticArmSetRodRotation(arm, target_rotation);

        if (RotationReached(arm, target_rotation, FLIP_ROTATION_TOLERANCE))
            *flip_state = FLIP_STATE_DONE;

        break;

    case FLIP_STATE_DONE:
    default:
        break;
    }
}
