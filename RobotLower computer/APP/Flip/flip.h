#ifndef __FLIP_H
#define __FLIP_H

#include "RoboticArm.h"
#include "bsp_config.h"

/*
 * 机械臂翻转/放置动作状态机。
 *
 * 本模块不直接发送 J60、GO 或舵机的底层通信帧。它只在适当时机调用
 * RoboticArmSetEndPosition() 与 RoboticArmSetRodRotation() 写入机械臂目标；
 * 实际的电机控制帧发送和反馈更新由 RoboticArmUpdateTask() 完成。
 *
 * 坐标约定：
 *   x：机械臂前后伸缩方向，x 增大表示向前伸出；
 *   z：机械臂升降方向，实际机械正负方向由 ROBOTICARM_LIFT_K 标定决定；
 *   rotation：翻转舵机的目标角度，单位 rad。
 *
 * 调用方式：顶层任务应周期调用 RoboticArmFlipMotion()。主动状态仅下发一次
 * 新目标，随后进入等待状态读取实际反馈，避免反复触发机械臂的 S 曲线规划。
 */

/* 翻转路径的参考原点。所有路径距离均在此基础上叠加，单位 m。 */
#define flip_start_x (0.0f)
#define flip_start_z (0.0f)
#define flip_start_rotation (0.0f)

/*
 * 翻转动作路径参数。
 * 调参时应先架空机械臂并逐段确认方向。不要同时修改这里的路径距离与
 * RoboticArm.h 中的运动学 K 值，否则难以区分路径误差和机构换算误差。
 */
#define FLIP_UP_DISTANCE_1 (0.48f)            /* 第一次上升量：起点 z 到抓取高度，单位 m。 */
#define FLIP_FORWARD_DISTANCE_1 (0.68f)       /* 第一次前伸量：起点 x 到抓取位置，单位 m。 */
#define FLIP_ROTATION_ANGLE_1 (BSP_PI)        /* 翻转角度：PI rad，即 180 度。 */
#define FLIP_UP_DISTANCE_2 (0.050f)           /* 翻转后额外上升量，用于避开障碍，单位 m。 */
#define FLIP_FORWARD_DISTANCE_2 (0.100f)      /* 翻转后回撤到的保留前伸量，单位 m。 */

/* 常规末端到位容差，单位 m。x、z 均到位才会推进下一状态。 */
#define FLIP_POSITION_TOLERANCE_X (0.005f)
#define FLIP_POSITION_TOLERANCE_Z (0.005f)

/*
 * 抓取阶段的宽松到位容差，单位 m。
 * 0.4 m 显著大于常规的 5 mm，到达该范围即可提前开始吸附测试，不能视为
 * 精确到达抓取点的判定。正式自动抓取前应依据吸盘有效接触范围重新标定。
 */
#define FLIP_POSITION_TOLERANCE_X_ALT (0.4f)
#define FLIP_POSITION_TOLERANCE_Z_ALT (0.4f)

/* 翻转角到位容差，单位 rad；PI2 / 180 约等于 2 度。 */
#define FLIP_ROTATION_TOLERANCE (PI2 / 180.0f)

/*
 * 翻转动作子状态。
 *
 * 每个主动状态负责写入一次目标，WAIT 状态只通过
 * PositionReached()/RotationReached() 判断真实反馈是否到位。
 *
 * 注意：当前 flip.c 中 FLIP_STATE_GRIP 会直接跳转至 FLIP_STATE_DONE。
 * 因而 ROTATE 至 BACK_AND_DOWN 的完整翻转/放置路径虽已预留且有实现，
 * 在当前测试流程中并不会进入。
 */
typedef enum
{
    FLIP_STATE_UP = 0,                 /* 下发第一次上升目标。 */
    FLIP_STATE_UP_WAIT,                /* 等待升至抓取高度。 */
    FLIP_STATE_FORWARD,                /* 下发前伸抓取目标，并启动真空气泵。 */
    FLIP_STATE_FORWARD_WAIT,           /* 等待进入抓取允许范围。 */
    FLIP_STATE_GRIP,                   /* 控制电磁阀吸住；当前测试后直接结束。 */
    FLIP_STATE_ROTATE,                 /* 预留：下发向后旋转 180 度目标。 */
    FLIP_STATE_ROTATE_WAIT,            /* 预留：等待旋转到位。 */
    FLIP_STATE_UP_AFTER_ROTATE,        /* 预留：翻转后额外上升。 */
    FLIP_STATE_UP_AFTER_ROTATE_WAIT,   /* 预留：等待额外上升到位。 */
    FLIP_STATE_BACK_AFTER_ROTATE,      /* 预留：保持高度回撤至放置点。 */
    FLIP_STATE_BACK_AFTER_ROTATE_WAIT, /* 预留：等待回撤到位。 */
    FLIP_STATE_RELEASE,                /* 预留：释放吸盘上的物块。 */
    FLIP_STATE_BACK_AND_DOWN,          /* 预留：回到翻转动作参考原点。 */
    FLIP_STATE_DONE                    /* 终止：关闭气泵并保持当前机械臂目标。 */
} FlipState_TypeDef;

/*
 * @brief 推进一次翻转动作状态机。
 * @param arm        机械臂实例。函数会写入 target_x、target_z、target_rotation。
 * @param flip_state 调用者保存的翻转状态；新一轮动作开始前应设为 FLIP_STATE_UP。
 *
 * 本函数不包含阻塞延时，也不直接更新电机反馈。CAN/UART 回调负责更新反馈，
 * RoboticArmUpdateTask() 负责周期执行 J60、GO 与舵机控制。
 */
void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, FlipState_TypeDef *flip_state);

#endif
