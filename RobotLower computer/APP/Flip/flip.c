#include "flip.h"
#include "GasPumpCLY.h"

void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, FlipState_TypeDef *flip_state)
{
    switch (*flip_state)
    {
    case FLIP_STATE_UP:
        arm->target_x = flip_start_x;
        arm->target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        *flip_state = FLIP_STATE_UP_WAIT;

        break;

    case FLIP_STATE_UP_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_FORWARD;

        break;

    case FLIP_STATE_FORWARD:
        arm->target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1;
        arm->target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        GasPumpOn();

        *flip_state = FLIP_STATE_FORWARD_WAIT;

        break;

    case FLIP_STATE_FORWARD_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, FLIP_POSITION_TOLERANCE_X_ALT, FLIP_POSITION_TOLERANCE_Z_ALT))
            *flip_state = FLIP_STATE_GRIP;

        break;

    case FLIP_STATE_GRIP:
        CLY_On();
        *flip_state = FLIP_STATE_DONE;

        break;

    case FLIP_STATE_ROTATE:
        arm->target_rotation = flip_start_rotation +
                          FLIP_ROTATION_ANGLE_1;

        RoboticArmSetRodRotation(arm, arm->target_rotation);

        if (RotationReached(arm, arm->target_rotation, FLIP_ROTATION_TOLERANCE))
            *flip_state = FLIP_STATE_UP_AFTER_ROTATE;

        break;

    case FLIP_STATE_UP_AFTER_ROTATE:
        arm->target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1;
        arm->target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1 + FLIP_UP_DISTANCE_2;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        if (PositionReached(arm, arm->target_x, arm->target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_BACK_AFTER_ROTATE;

        break;

    case FLIP_STATE_BACK_AFTER_ROTATE:
        arm->target_x = flip_start_x + FLIP_FORWARD_DISTANCE_2;
        arm->target_z = flip_start_z + FLIP_UP_DISTANCE_1 + FLIP_UP_DISTANCE_2;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        if (PositionReached(arm, arm->target_x, arm->target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_RELEASE;

        break;

    case FLIP_STATE_RELEASE:
        CLY_Off();
        *flip_state = FLIP_STATE_BACK_AND_DOWN;

        break;

    case FLIP_STATE_BACK_AND_DOWN:
        arm->target_x = flip_start_x;
        arm->target_z = flip_start_z;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        if (PositionReached(arm, arm->target_x, arm->target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_DONE;

        break;

    case FLIP_STATE_DONE:
        GasPumpOff();
        break;
    default:
        break;
    }
}
