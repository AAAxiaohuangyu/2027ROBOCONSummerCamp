#ifndef __FLIP_H
#define __FLIP_H

#include "RoboticArm.h"
#include "chassis.h"
#include "bsp_config.h"

/* 吸盘初始左边即旋转角度 */
#define flip_start_x (0.0f)
#define flip_start_z (0.0f)
#define flip_start_rotation (BSP_PI)

/* 动作距离及角度，根据实际机械结构修改；旋转量与RoboticArmSetRodRotation/
   RotationReached保持一致，单位为rad(电机转角直接相等，见RoboticArm.h) */
#define FLIP_UP_DISTANCE_1 (0.485f)
#define FLIP_FORWARD_DISTANCE_1 (0.25f)
#define FLIP_FORWARD_DISTANCE_2 (0.31f)
#define FLIP_UP_DISTANCE_2 (0.1f)
#define FLIP_ROTATION_ANGLE_1 (-2.0f)
#define FLIP_ROTATION_ANGLE_2 (-0.6f)

#define FLIP_POSITION_TOLERANCE_X (0.02f)
#define FLIP_POSITION_TOLERANCE_Z (0.015f)
/*前进过程中开启电磁阀抓取的时机*/
#define FLIP_POSITION_TOLERANCE_X_ALT (0.05f)
#define FLIP_POSITION_TOLERANCE_Z_ALT (0.05f)

#define FLIP_ROTATION_TOLERANCE (PI2 / 180.0f) /* 2度 */

typedef enum
{
    FLIP_STATE_UP = 0,
    FLIP_STATE_UP_WAIT,
    FLIP_STATE_FORWARD,
    FLIP_STATE_FORWARD_WAIT,
    FLIP_STATE_UP2,
    FLIP_STATE_UP2_WAIT,
    FLIP_STATE_CHASSIS_MOVE_RIGHT,
    FLIP_STATE_CHASSIS_MOVE_RIGHT_WAIT,
    FLIP_STATE_FORWARD2,
    FLIP_STATE_FORWARD2_WAIT,
    FLIP_STATE_ROTATE,
    FLIP_STATE_CHASSIS_MOVE_LEFT,
    FLIP_STATE_CHASSIS_MOVE_LEFT_WAIT,
    FLIP_STATE_ROTATE2,
    FLIP_STATE_RELEASE,
    FLIP_STATE_CHASSIS_MOVE_RIGHT2,
    FLIP_STATE_CHASSIS_MOVE_RIGHT2_WAIT,
    FLIP_STATE_ROBOTICARM_BACK,
    FLIP_STATE_DONE
} FlipState_TypeDef;

typedef struct
{
    FlipState_TypeDef state;
    uint8_t active;
    uint8_t complete;
} Flip_TypeDef;

void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, Chassis_TypeDef *chassis, FlipState_TypeDef *flip_state,uint8_t *flip_active);

#endif
