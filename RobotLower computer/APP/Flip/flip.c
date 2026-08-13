#include "flip.h"

static uint8_t FlipPositionReached(RoboticArm_TypeDef *arm,
                                   float target_x,
                                   float target_z)
{
    float dx = arm->end_x - target_x;
    float dz = arm->end_z - target_z;

    if (dx < 0.0f)
    {
        dx = -dx;
    }

    if (dz < 0.0f)
    {
        dz = -dz;
    }

    return (dx <= FLIP_POSITION_TOLERANCE &&
            dz <= FLIP_POSITION_TOLERANCE);
}

static uint8_t FlipRotationReached(RoboticArm_TypeDef *arm,
                                   float target_rotation)
{
    float error = arm->rod_rotation - target_rotation;

    if (error < 0.0f)
    {
        error = -error;
    }

    return (error <= FLIP_ROTATION_TOLERANCE);
}

void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, FlipState_TypeDef *flip_state)
{
    float target_x;
    float target_z;
    float target_rotation;

    if (*flip_state == FLIP_STATE_DONE)
    {
        *flip_state = FLIP_STATE_UP;
    }

    switch (*flip_state)
    {
    case FLIP_STATE_UP:
        target_x = flip_start_x;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, target_x, target_z);

        if (FlipPositionReached(arm, target_x, target_z))
            *flip_state = FLIP_STATE_FORWARD;

        break;

    case FLIP_STATE_FORWARD:
        target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, target_x, target_z);

        if (FlipPositionReached(arm, target_x, target_z))
            *flip_state = FLIP_STATE_GRIP;

        break;

    case FLIP_STATE_GRIP:
        RoboticArmGripMotion();
        *flip_state = FLIP_STATE_ROTATE;

        break;

    case FLIP_STATE_ROTATE:
        target_rotation = flip_start_rotation +
                          FLIP_ROTATION_ANGLE_1;

        RoboticArmSetRodRotation(arm, target_rotation);

        if (FlipRotationReached(arm, target_rotation))
            *flip_state = FLIP_STATE_FORWARD_AFTER_ROTATE;

        break;

    case FLIP_STATE_FORWARD_AFTER_ROTATE:
        target_x = flip_start_x +
                   FLIP_FORWARD_DISTANCE_1 +
                   FLIP_FORWARD_DISTANCE_2;
        target_z = flip_start_z +
                   FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, target_x, target_z);

        if (FlipPositionReached(arm, target_x, target_z))
            *flip_state = FLIP_STATE_RELEASE;

        break;

    case FLIP_STATE_RELEASE:
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

        RoboticArmSetEndPosition(arm, target_x, target_z);

        if (FlipPositionReached(arm, target_x, target_z))
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

        RoboticArmSetEndPosition(arm, target_x, target_z);

        if (FlipPositionReached(arm, target_x, target_z))
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

        RoboticArmSetEndPosition(arm, target_x, target_z);

        if (FlipPositionReached(arm, target_x, target_z))
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

        RoboticArmSetEndPosition(arm, target_x, target_z);

        if (FlipPositionReached(arm, target_x, target_z))
            *flip_state = FLIP_STATE_ROTATE_BACK;

        break;

    case FLIP_STATE_ROTATE_BACK:
        target_rotation = flip_start_rotation +
                          FLIP_ROTATION_ANGLE_1 +
                          FLIP_ROTATION_ANGLE_2;

        RoboticArmSetRodRotation(arm, target_rotation);

        if (FlipRotationReached(arm, target_rotation))
            *flip_state = FLIP_STATE_DONE;

        break;

    case FLIP_STATE_DONE:
    default:
        break;
    }
}
