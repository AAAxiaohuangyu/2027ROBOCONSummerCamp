#ifndef __FLIP_H
#define __FLIP_H

#include "RoboticArm.h"

/* 吸盘初始左边即旋转角度 */
#define flip_start_x (0.0f)
#define flip_start_z (0.0f)
#define flip_start_rotation (0.0f)

/* 动作距离及角度，根据实际机械结构修改；旋转量与RoboticArmSetRodRotation/
   RotationReached保持一致，单位为rad(电机转角直接相等，见RoboticArm.h) */
#define FLIP_UP_DISTANCE_1 (0.050f)
#define FLIP_FORWARD_DISTANCE_1 (0.100f)
#define FLIP_ROTATION_ANGLE_1 (PI2 / 2.0f) /* 180度 */
#define FLIP_FORWARD_DISTANCE_2 (0.100f)
#define FLIP_FORWARD_DISTANCE_3 (0.100f)
#define FLIP_UP_DISTANCE_2 (0.050f)
#define FLIP_BACKWARD_DISTANCE (0.100f)
#define FLIP_DOWN_DISTANCE (0.050f)
#define FLIP_ROTATION_ANGLE_2 (PI2 / 2.0f) /* 180度 */

#define FLIP_POSITION_TOLERANCE_X (0.005f)
#define FLIP_POSITION_TOLERANCE_Z (0.005f)
#define FLIP_ROTATION_TOLERANCE (PI2 / 180.0f) /* 2度 */

typedef enum
{
    FLIP_STATE_UP = 0,             /* 从初始位置抬升。 */
    FLIP_STATE_FORWARD,            /* 移动至 KFS 上方。 */
    FLIP_STATE_GRIP,               /* 吸附 KFS。 */
    FLIP_STATE_ROTATE,             /* 第一次旋转，完成翻转。 */
    FLIP_STATE_FORWARD_AFTER_ROTATE, /* 移动至释放位置。 */
    FLIP_STATE_RELEASE,            /* 释放已翻转的 KFS。 */
    FLIP_STATE_FORWARD_AFTER_RELEASE, /* 离开释放位置。 */
    FLIP_STATE_UP_AFTER_RELEASE,   /* 抬升至回程安全高度。 */
    FLIP_STATE_BACK,               /* 后退至回落位置。 */
    FLIP_STATE_DOWN,               /* 下降至复位高度。 */
    FLIP_STATE_ROTATE_BACK,        /* 第二次旋转，恢复初始朝向。 */
    FLIP_STATE_DONE                /* 翻转流程结束。 */
} FlipState_TypeDef;

__weak void RoboticArmGripMotion(void);
__weak void RoboticArmReleaseMotion(void);
void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, FlipState_TypeDef *flip_state);

#endif
