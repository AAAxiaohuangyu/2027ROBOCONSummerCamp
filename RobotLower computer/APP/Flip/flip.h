#ifndef __FLIP_H
#define __FLIP_H

#include "RoboticArm.h"
#include "bsp_config.h"

/* 吸盘初始左边即旋转角度 */
#define flip_start_x (0.0f)
#define flip_start_z (0.0f)
#define flip_start_rotation (0.0f)

/* 动作距离及角度，根据实际机械结构修改；旋转量与RoboticArmSetRodRotation/
   RotationReached保持一致，单位为rad(电机转角直接相等，见RoboticArm.h) */
#define FLIP_UP_DISTANCE_1 (0.050f)
#define FLIP_FORWARD_DISTANCE_1 (0.100f)
#define FLIP_ROTATION_ANGLE_1 (BSP_PI) /* 180度 */
#define FLIP_UP_DISTANCE_2 (0.050f)
#define FLIP_FORWARD_DISTANCE_2 (0.100f)

#define FLIP_POSITION_TOLERANCE_X (0.005f)
#define FLIP_POSITION_TOLERANCE_Z (0.005f)
/*前进过程中开启电磁阀抓取的时机*/
#define FLIP_POSITION_TOLERANCE_X_ALT (0.005f)
#define FLIP_POSITION_TOLERANCE_Z_ALT (0.005f)

#define FLIP_ROTATION_TOLERANCE (PI2 / 180.0f) /* 2度 */

typedef enum
{
    FLIP_STATE_UP = 0,
    FLIP_STATE_FORWARD,
    FLIP_STATE_GRIP,
    FLIP_STATE_ROTATE,
    FLIP_STATE_UP_AFTER_ROTATE,
    FLIP_STATE_BACK_AFTER_ROTATE,
    FLIP_STATE_RELEASE,
    FLIP_STATE_BACK_AND_DOWN,
    FLIP_STATE_DONE
} FlipState_TypeDef;

void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, FlipState_TypeDef *flip_state);

#endif
