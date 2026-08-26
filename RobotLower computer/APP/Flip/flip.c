#include "flip.h"
#include "GasPumpCLY.h"

void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, FlipState_TypeDef *flip_state)
{
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

        GasPumpOn();

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X_ALT, FLIP_POSITION_TOLERANCE_Z_ALT))
            *flip_state = FLIP_STATE_GRIP;

        break;

    case FLIP_STATE_GRIP:
        CLY_On();

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_ROTATE;

        break;

    case FLIP_STATE_ROTATE:
        target_rotation = flip_start_rotation +
                          FLIP_ROTATION_ANGLE_1;

        RoboticArmSetRodRotation(arm, target_rotation);

        if (RotationReached(arm, target_rotation, FLIP_ROTATION_TOLERANCE))
            *flip_state = FLIP_STATE_UP_AFTER_ROTATE;

        break;

    case FLIP_STATE_UP_AFTER_ROTATE:
        target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1 + FLIP_UP_DISTANCE_2;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_BACK_AFTER_ROTATE;

        break;

    case FLIP_STATE_BACK_AFTER_ROTATE:
        target_x = flip_start_x + FLIP_FORWARD_DISTANCE_2;
        target_z = flip_start_z + FLIP_UP_DISTANCE_1 + FLIP_UP_DISTANCE_2;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_RELEASE;

        break;

    case FLIP_STATE_RELEASE:
        CLY_Off();
        *flip_state = FLIP_STATE_BACK_AND_DOWN;

        break;

    case FLIP_STATE_BACK_AND_DOWN:
        target_x = flip_start_x;
        target_z = flip_start_z;

        RoboticArmSetEndPosition(arm, target_x, target_z, GravityCompensationLift);

        if (PositionReached(arm, target_x, target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_DONE;

        break;

    case FLIP_STATE_DONE:
        GasPumpOff();
        break;
    default:
        break;
    }
}
