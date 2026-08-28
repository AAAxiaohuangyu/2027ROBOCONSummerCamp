#include "flip.h"
#include "GasPumpCLY.h"
#include "cmsis_os2.h"
#include "chassis.h"

void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, Chassis_TypeDef *chassis, FlipState_TypeDef *flip_state)
{
    switch (*flip_state)
    {
    case FLIP_STATE_UP:
        RoboticArmSetRodRotation(arm,BSP_PI);

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
        CLY_On();

        *flip_state = FLIP_STATE_FORWARD_WAIT;

        break;

    case FLIP_STATE_FORWARD_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
        {
            *flip_state = FLIP_STATE_UP2;
        }

        break;

    case FLIP_STATE_UP2:
        arm->target_x = flip_start_x +
                        FLIP_FORWARD_DISTANCE_1;
        arm->target_z = flip_start_z +
                        FLIP_UP_DISTANCE_1+FLIP_UP_DISTANCE_2;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);
        *flip_state = FLIP_STATE_UP2_WAIT;
        break;

    case FLIP_STATE_UP2_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_CHASSIS_MOVE_RIGHT;
        break;

    case FLIP_STATE_CHASSIS_MOVE_RIGHT:
        ChassisSetTranslation(chassis, 0.0f, -0.6f);
        //ChassisSetTranslation(chassis, 3.38f, -0.5f);
        *flip_state = FLIP_STATE_CHASSIS_MOVE_RIGHT_WAIT;

        break;

    case FLIP_STATE_CHASSIS_MOVE_RIGHT_WAIT:
        if (ChassisTranslationReached(chassis,0.02f))
            *flip_state = FLIP_STATE_FORWARD2;

        break;

    case FLIP_STATE_FORWARD2:
        arm->target_x = flip_start_x +
                        FLIP_FORWARD_DISTANCE_1 + FLIP_FORWARD_DISTANCE_2;
        arm->target_z = flip_start_z +
                        FLIP_UP_DISTANCE_1 + FLIP_UP_DISTANCE_2;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);
        *flip_state = FLIP_STATE_FORWARD2_WAIT;

        break;

    case FLIP_STATE_FORWARD2_WAIT:
        if (PositionReached(arm, arm->target_x, arm->target_z, FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
            *flip_state = FLIP_STATE_ROTATE;

        break;

    case FLIP_STATE_ROTATE:
        arm->target_rotation = flip_start_rotation +
                          FLIP_ROTATION_ANGLE_1;

        RoboticArmSetRodRotation(arm, arm->target_rotation);

        if (RotationReached(arm, arm->target_rotation, FLIP_ROTATION_TOLERANCE))
            *flip_state = FLIP_STATE_DONE;

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
        //GasPumpOff();
        break;
    default:
        break;
    }
}
