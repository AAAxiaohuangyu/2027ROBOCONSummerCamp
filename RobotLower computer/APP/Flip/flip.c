#include "flip.h"
#include "GasPumpCLY.h"

/*
 * 翻转动作状态机的单周期执行函数。
 *
 * 这里仅决定机械臂目标与气路状态，不能替代 RoboticArmUpdate()；J60、GO、
 * 舵机的控制帧发送及反馈位置刷新，仍由独立的机械臂 RTOS 任务完成。
 */
void RoboticArmFlipMotion(RoboticArm_TypeDef *arm, FlipState_TypeDef *flip_state)
{
    switch (*flip_state)
    {
    case FLIP_STATE_UP:
        /* 第一步：从翻转参考原点上升至抓取高度，保持 x 位于起点。 */
        arm->target_x = flip_start_x;
        arm->target_z = flip_start_z + FLIP_UP_DISTANCE_1;

        /* 同时写入 J60/GO 的位置目标；重力前馈由机械臂模块统一提供。 */
        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        /* 目标仅下发一次，后续周期由 UP_WAIT 读取实际反馈。 */
        *flip_state = FLIP_STATE_UP_WAIT;
        break;

    case FLIP_STATE_UP_WAIT:
        /* x、z 都进入常规容差后，才允许开始前伸，防止斜向运动发生干涉。 */
        if (PositionReached(arm, arm->target_x, arm->target_z,
                            FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
        {
            *flip_state = FLIP_STATE_FORWARD;
        }
        break;

    case FLIP_STATE_FORWARD:
        /* 第二步：保持抓取高度，向前伸至物块或待翻转机构的位置。 */
        arm->target_x = flip_start_x + FLIP_FORWARD_DISTANCE_1;
        arm->target_z = flip_start_z + FLIP_UP_DISTANCE_1;

        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        /* 前伸阶段即启动气泵，让吸盘在接触目标时具备真空。 */
        GasPumpOn();

        *flip_state = FLIP_STATE_FORWARD_WAIT;
        break;

    case FLIP_STATE_FORWARD_WAIT:
        /* 当前使用宽松抓取范围；这是提前吸附测试，不是精确末端到位。 */
        if (PositionReached(arm, arm->target_x, arm->target_z,
                            FLIP_POSITION_TOLERANCE_X_ALT, FLIP_POSITION_TOLERANCE_Z_ALT))
        {
            *flip_state = FLIP_STATE_GRIP;
        }
        break;

    case FLIP_STATE_GRIP:
        /* CLY_On() 将电磁阀切换至吸住状态，气泵已在前伸阶段开启。 */
        CLY_On();

        /*
         * 吸住后继续执行完整 Flip 流程：向后旋转、额外抬升、回撤、释放，
         * 最后同步缩回并下降至翻转动作的参考原点。
         */
        *flip_state = FLIP_STATE_ROTATE;
        break;

    case FLIP_STATE_ROTATE:
        /* 预留完整流程：下发向后旋转 180 度的舵机目标。 */
        arm->target_rotation = flip_start_rotation + FLIP_ROTATION_ANGLE_1;
        RoboticArmSetRodRotation(arm, arm->target_rotation);

        /* 当前直接检查旋转到位；ROTATE_WAIT 是未来拆分状态时的预留项。 */
        if (RotationReached(arm, arm->target_rotation, FLIP_ROTATION_TOLERANCE))
        {
            *flip_state = FLIP_STATE_UP_AFTER_ROTATE;
        }
        break;

    case FLIP_STATE_UP_AFTER_ROTATE:
        /* 预留完整流程：翻转后额外抬升，避免回撤时与物块或机构干涉。 */
        arm->target_x = flip_start_x + FLIP_FORWARD_DISTANCE_1;
        arm->target_z = flip_start_z + FLIP_UP_DISTANCE_1 + FLIP_UP_DISTANCE_2;
        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        if (PositionReached(arm, arm->target_x, arm->target_z,
                            FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
        {
            *flip_state = FLIP_STATE_BACK_AFTER_ROTATE;
        }
        break;

    case FLIP_STATE_BACK_AFTER_ROTATE:
        /* 预留完整流程：保持当前高度，缩回到放置位置上方。 */
        arm->target_x = flip_start_x + FLIP_FORWARD_DISTANCE_2;
        arm->target_z = flip_start_z + FLIP_UP_DISTANCE_1 + FLIP_UP_DISTANCE_2;
        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        if (PositionReached(arm, arm->target_x, arm->target_z,
                            FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
        {
            *flip_state = FLIP_STATE_RELEASE;
        }
        break;

    case FLIP_STATE_RELEASE:
        /* 预留完整流程：切换电磁阀释放物块，再开始回到参考原点。 */
        CLY_Off();
        *flip_state = FLIP_STATE_BACK_AND_DOWN;
        break;

    case FLIP_STATE_BACK_AND_DOWN:
        /* 预留完整流程：同时回缩、下降至翻转动作的参考原点。 */
        arm->target_x = flip_start_x;
        arm->target_z = flip_start_z;
        RoboticArmSetEndPosition(arm, arm->target_x, arm->target_z, GravityCompensationLift);

        if (PositionReached(arm, arm->target_x, arm->target_z,
                            FLIP_POSITION_TOLERANCE_X, FLIP_POSITION_TOLERANCE_Z))
        {
            *flip_state = FLIP_STATE_DONE;
        }
        break;

    case FLIP_STATE_DONE:
        /*
         * 测试结束后关闭真空气泵。这里不调用 CLY_Off()，电磁阀会保持 GRIP
         * 阶段设置的状态；是否符合实际气路，需要依据电磁阀型号和气路确认。
         */
        GasPumpOff();
        break;

    default:
        /* 非法状态不强制复位，便于调试时在 Watch 中定位异常状态来源。 */
        break;
    }
}
