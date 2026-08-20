#include "flip.h"

void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, FlipState_TypeDef *flip_state)
{
    float target_x;
    float target_z;
    float target_rotation;

    switch (*flip_state)
    {
    case FLIP_STATE_UP:
        /* 从初始位置抬起，为水平移动留出安全间隙。 */
        target_x = flip_start_x;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_FORWARD;

        break;

    case FLIP_STATE_FORWARD:
        /* 移动至待翻转 KFS 的正上方。 */
        target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_GRIP;

        break;

    case FLIP_STATE_GRIP:
        /* 开启吸盘，固定待翻转的 KFS。 */
        RoboticArmGripMotion();
        *flip_state = FLIP_STATE_ROTATE;

        break;

    case FLIP_STATE_ROTATE:
        /* 旋转吸盘杆，将 KFS 翻转 180 度。 */
        target_rotation = flip_start_rotation +
                          FLIP_ROTATION_ANGLE_1;

        RoboticArmSetRodRotation(arm, target_rotation);

        if (RotationReached(arm, target_rotation, FLIP_ROTATION_TOLERANCE))
            *flip_state = FLIP_STATE_FORWARD_AFTER_ROTATE;

        break;

    case FLIP_STATE_FORWARD_AFTER_ROTATE:
        /* 翻转后向前移动至释放位置。 */
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
        /* 关闭吸盘，将已翻转的 KFS 放下。 */
        RoboticArmReleaseMotion();
        *flip_state = FLIP_STATE_FORWARD_AFTER_RELEASE;

        break;

    case FLIP_STATE_FORWARD_AFTER_RELEASE:
        /* 离开释放点，避免后续抬升时干涉 KFS。 */
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
        /* 抬升至回程安全高度。 */
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
        /* 后退至机械臂回落位置。 */
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
        /* 降低末端，恢复旋转前的空间关系。 */
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
        /* 再旋转 180 度，使吸盘杆回到初始朝向。 */
        target_rotation = flip_start_rotation +
                          FLIP_ROTATION_ANGLE_1 +
                          FLIP_ROTATION_ANGLE_2;

        RoboticArmSetRodRotation(arm, target_rotation);

        if (RotationReached(arm, target_rotation, FLIP_ROTATION_TOLERANCE))
            *flip_state = FLIP_STATE_DONE;

        break;

    case FLIP_STATE_DONE:
        /* 翻转流程完成，主控负责切换到下一任务。 */
    default:
        break;
    }
}
