#include "Core.h"
#include "bsp_config.h"
#include "chassis_config.h"
#include "ControlAlgorithm.h"
#include "cmsis_os2.h"
#include "usart.h"
#include <string.h>

Robot_TypeDef Robot = {0};

/* MOVE_5~MOVE_8阶段x跟踪固定切到第三套参数组,MOVE_6~MOVE_8阶段y轴固定切到备用
   速度规划/跟踪参数组;这两次切换本应分别发生在START_MOVE_5/START_MOVE_6状态,
   若该阶段被KFS_DIFF跳过则由RobotSkipMove补做一次,以保证后续阶段仍使用正确参数 */
static void RobotApplyMove5Params(void)
{
    PIDInit(&Robot.chassis.displacement_plan.translation_x.track_pid,
            CHASSIS_TRACK_TRANSLATION_X_ALT2_KP,
            CHASSIS_TRACK_TRANSLATION_X_ALT2_KI,
            CHASSIS_TRACK_TRANSLATION_X_ALT2_KD,
            CHASSIS_TRACK_TRANSLATION_X_ALT2_MAX_OUT,
            CHASSIS_TRACK_TRANSLATION_X_ALT2_MAX_IOUT);
}

static void RobotApplyMove6Params(void)
{
    SpeedPlanInit(&Robot.chassis.displacement_plan.translation_y,
                  CHASSIS_PLAN_TRANSLATION_Y_ALT_MAX_ACCEL_MPS2,
                  CHASSIS_PLAN_TRANSLATION_Y_ALT_MAX_SPEED_MPS,
                  CHASSIS_PLAN_TRANSLATION_Y_ALT_MAX_JERK_MPS3,
                  CHASSIS_TRACK_TRANSLATION_DEADBAND_M);
    PIDInit(&Robot.chassis.displacement_plan.translation_y.track_pid,
            CHASSIS_TRACK_TRANSLATION_Y_ALT_KP,
            CHASSIS_TRACK_TRANSLATION_Y_ALT_KI,
            CHASSIS_TRACK_TRANSLATION_Y_ALT_KD,
            CHASSIS_TRACK_TRANSLATION_Y_ALT_MAX_OUT,
            CHASSIS_TRACK_TRANSLATION_Y_ALT_MAX_IOUT);
}

/* MOVE_9(上斜坡)阶段x跟踪切到第四套参数组(不重新调用ChassisSetTranslation,
   仅切换跟踪增益) */
static void RobotApplyMove9Params(void)
{
    PIDInit(&Robot.chassis.displacement_plan.translation_x.track_pid,
            CHASSIS_TRACK_TRANSLATION_X_ALT3_KP,
            CHASSIS_TRACK_TRANSLATION_X_ALT3_KI,
            CHASSIS_TRACK_TRANSLATION_X_ALT3_KD,
            CHASSIS_TRACK_TRANSLATION_X_ALT3_MAX_OUT,
            CHASSIS_TRACK_TRANSLATION_X_ALT3_MAX_IOUT);
}

/* Robot.vision.KFS_DIFF为1~5时分别跳过阶段5~9(即KFS_DIFF+4那一段的START/WAIT状态,
   直接进入下一阶段;跳过阶段9则直接进入ROBOT_STATE_MANUAL),其余KFS_DIFF值不处理 */
static RobotState_TypeDef RobotSkipMove(uint8_t move_num, RobotState_TypeDef next_state)
{
    if (Robot.vision.KFS_DIFF > 0 && Robot.vision.KFS_DIFF < 5 && move_num == (uint8_t)(Robot.vision.KFS_DIFF + 4))
    {
        if (move_num == 5)
            RobotApplyMove5Params();
        else if (move_num == 6)
            RobotApplyMove6Params();

        return (move_num == 9) ? ROBOT_STATE_MANUAL : (RobotState_TypeDef)(next_state + 2);
    }
    return next_state;
}

void RobotInit(void)
{
    HAL_Delay(1000);
    ChassisInit(&Robot.chassis, CHASSIS_FDCAN_HANDLE, CHASSIS_CTRL_ID);
    FDCANStandardInit(Robot.chassis.drive.motor_group.FDCAN_Handle,
                      M3508_FEEDBACK_ID_BASE + M3508_ID_MIN,
                      M3508_FEEDBACK_ID_BASE + M3508_ID_MAX); /* 过滤器0 -> RXFIFO0 */

    EncoderInit(&Robot.encoder,
                ENCODER_X_FDCAN_HANDLE, ENCODER_X_NODE_ID,
                ENCODER_Y_FDCAN_HANDLE, ENCODER_Y_NODE_ID); /* 过滤器0 -> RXFIFO1 */

    Yis512Init(&Robot.yis512); /* 扩展过滤器0 -> RXFIFO0 */

    /* 开启UART9空闲线DMA接收;之后每帧到达都由HAL_UARTEx_RxEventCallback(bsp_callback.c)
       中断驱动解析,持续更新Robot.zigbee.explained_data,不需要额外的周期任务 */
    Zigbee_Init(&Robot.zigbee);

    Vision_Init(&Robot.vision);

    RoboticArmInit(&Robot.roboticarm, ROBOTICARM_LIFT_FDCAN_HANDLE, ROBOTICARM_LIFT_ID, ROBOTICARM_FORWARD_UART_HANDLE, ROBOTICARM_FORWARD_ID, SERVO_PWM_TIMER_HANDLE, SERVO_PWM_CHANNEL);
    uint16_t lift_feedback_base = (uint16_t)((J60_RESPONSE_FEEDBACK << J60_CAN_ID_RESPONSE_SHIFT) |
                                             (J60_CMD_CONTROL << J60_CAN_ID_COMMAND_SHIFT));
    FDCANStandardInit(Robot.roboticarm.lift_motor.FDCAN_Handle,
                      lift_feedback_base + J60_ID_MIN,
                      lift_feedback_base + J60_ID_MAX);

    RoboticArmEnable(&Robot.roboticarm);
}

void RobotChassisUpdateTask(void *argument)
{
    (void)argument;
    ChassisUpdate(&Robot.chassis, &Robot.yis512); /* ChassisInit后velocity=0/mode=1,持续下发0电流控制帧触发电调反馈;函数内含while(1),此任务不会返回 */
}

void RobotEncoderUpdateTask(void *argument)
{
    (void)argument;
    EncoderUpdate(&Robot.encoder); /* 周期发位置请求触发encoder应答;函数内含while(1),此任务不会返回 */
}

void RobotRoboticArmUpdateTask(void *argument)
{
    (void)argument;
    RoboticArmUpdate(&Robot.roboticarm); /* 周期下发J60/GO控制帧触发反馈;函数内含while(1),此任务不会返回 */
}

void RobotFlipUpdateTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        if (Robot.flip.active)
        {
            RoboticArmFlipMotion(&Robot.roboticarm, &Robot.chassis, &Robot.flip.state, &Robot.flip.complete);
        }
        osDelay(ROBOT_STATE_UPDATE_PERIOD_MS);
    }
}

void RobotPickupUpdateTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        if (Robot.pickup.active)
        {
            switch (Robot.pickup.action)
            {
            case PICKUP_TASK_ACTION_STORE_LOW_LOW:
                RoboticArmPickupStoreLowMotion(&Robot.roboticarm, &Robot.pickup.state, PICKUP_HEIGHT_LOW,&Robot.pickup.complete);
                break;

            case PICKUP_TASK_ACTION_STORE_LOW_HIGH:
                RoboticArmPickupStoreLowMotion(&Robot.roboticarm, &Robot.pickup.state, PICKUP_HEIGHT_HIGH, &Robot.pickup.complete);
                break;

            case PICKUP_TASK_ACTION_STORE_HIGH_LOW:
                RoboticArmPickupStoreHighMotion(&Robot.roboticarm, &Robot.pickup.state, PICKUP_HEIGHT_LOW, &Robot.pickup.complete);
                break;

            case PICKUP_TASK_ACTION_STORE_HIGH_HIGH:
                RoboticArmPickupStoreHighMotion(&Robot.roboticarm, &Robot.pickup.state, PICKUP_HEIGHT_HIGH, &Robot.pickup.complete);
                break;

            case PICKUP_TASK_ACTION_HOLD_LOW:
                RoboticArmPickupHoldMotion(&Robot.roboticarm, &Robot.pickup.state, PICKUP_HEIGHT_LOW, &Robot.pickup.complete);
                break;

            case PICKUP_TASK_ACTION_HOLD_HIGH:
                RoboticArmPickupHoldMotion(&Robot.roboticarm, &Robot.pickup.state, PICKUP_HEIGHT_HIGH, &Robot.pickup.complete);
                break;

            default:
                break;
            }
        }

        osDelay(ROBOT_STATE_UPDATE_PERIOD_MS);
    }
}

void RobotStateUpdateTask(void *argument)
{
    Robot.state = ROBOT_STATE_PICKUP;
    Robot.vision.KFS_DIFF = 0;

    Robot.pickup.state = PICKUP_STATE_RAISE;

    (void)argument;

    for (;;)
    {
        if (Robot.zigbee.rx_data.command.mode == 1)
        {
            Robot.state = ROBOT_STATE_MANUAL;
        }

        switch (Robot.state)
        {
        case ROBOT_STATE_START_MOVE_1:
            osDelay(500);
            ChassisSetTranslation(&Robot.chassis, 0.8f, -0.65f);
            Robot.state = ROBOT_STATE_WAIT_MOVE_1;
            break;

        case ROBOT_STATE_WAIT_MOVE_1:
        {
            float corner_tolerance = ROBOT_STATE_CORNER_BLEND_K *
                                     SpeedPlanDecelDistance(&Robot.chassis.displacement_plan.translation_x);
            if (corner_tolerance < ROBOT_CHASSIS_POSITION_TOLERANCE_M)
                corner_tolerance = ROBOT_CHASSIS_POSITION_TOLERANCE_M;

            if (ChassisTranslationReached(&Robot.chassis, corner_tolerance))
                Robot.state = ROBOT_STATE_START_MOVE_2;
            break;
        }

        case ROBOT_STATE_START_MOVE_2:
            ChassisSetTranslation(&Robot.chassis, 3.38f, -0.65f);
            Robot.state = ROBOT_STATE_WAIT_MOVE_2;
            break;

        case ROBOT_STATE_WAIT_MOVE_2:
        {
            float corner_tolerance = (ROBOT_STATE_CORNER_BLEND_K + 0.3f) *
                                     SpeedPlanDecelDistance(&Robot.chassis.displacement_plan.translation_x);
            if (corner_tolerance < ROBOT_CHASSIS_POSITION_TOLERANCE_M)
                corner_tolerance = ROBOT_CHASSIS_POSITION_TOLERANCE_M;
            if (ChassisTranslationReached(&Robot.chassis, corner_tolerance))
                Robot.state = ROBOT_STATE_START_MOVE_3;
            break;
        }

        case ROBOT_STATE_START_MOVE_3:
        {
            ChassisSetTranslation(&Robot.chassis, 3.38f, -0.1f);
            Robot.state = ROBOT_STATE_WAIT_MOVE_3;
            break;
        }

        case ROBOT_STATE_WAIT_MOVE_3:
        {
            if (ChassisTranslationReached(&Robot.chassis, 4.0f * ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                Robot.state = ROBOT_STATE_FLIIP;
            break;
        }

        case ROBOT_STATE_FLIIP:
            Robot.flip.active = 1;
            if (Robot.flip.complete == 1)
            {
                Robot.state = ROBOT_STATE_START_MOVE_4;
            }
            break;

        case ROBOT_STATE_START_MOVE_4:
        {
            Robot.flip.active = 0;
            osDelay(100);
            /* MOVE_4/WAIT_MOVE_4阶段x方向回退幅度大,临时切到备用跟踪参数组,
               离开该阶段(START_MOVE_5)后切回默认组 */
            PIDInit(&Robot.chassis.displacement_plan.translation_x.track_pid,
                    CHASSIS_TRACK_TRANSLATION_X_ALT_KP,
                    CHASSIS_TRACK_TRANSLATION_X_ALT_KI,
                    CHASSIS_TRACK_TRANSLATION_X_ALT_KD,
                    CHASSIS_TRACK_TRANSLATION_X_ALT_MAX_OUT,
                    CHASSIS_TRACK_TRANSLATION_X_ALT_MAX_IOUT);
            ChassisSetTranslation(&Robot.chassis, 0.38f, -2.1f);
            Robot.state = ROBOT_STATE_WAIT_MOVE_4;
            break;
        }

        case ROBOT_STATE_WAIT_MOVE_4:
        {
            float corner_tolerance = (ROBOT_STATE_CORNER_BLEND_K)*SpeedPlanDecelDistance(&Robot.chassis.displacement_plan.translation_x);
            if (corner_tolerance < ROBOT_CHASSIS_POSITION_TOLERANCE_M)
                corner_tolerance = ROBOT_CHASSIS_POSITION_TOLERANCE_M;
            if (ChassisTranslationReached(&Robot.chassis, corner_tolerance))
                Robot.state = ROBOT_STATE_DONE;
                //Robot.state = RobotSkipMove(5, ROBOT_STATE_START_MOVE_5);
            break;
        }

        case ROBOT_STATE_START_MOVE_5:
        {
            /* 离开MOVE_4/WAIT_MOVE_4,MOVE_5~MOVE_8阶段x跟踪切到第三套参数组;
               y轴的备用速度规划/跟踪参数组从MOVE_6起才切换 */
            RobotApplyMove5Params();
            ChassisSetTranslation(&Robot.chassis, 0.38f, -3.42f);
            Robot.state = ROBOT_STATE_WAIT_MOVE_5;
            break;
        }

        case ROBOT_STATE_WAIT_MOVE_5:
        {
            if (ChassisTranslationReached(&Robot.chassis, 3.0f * ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                Robot.state = RobotSkipMove(6, ROBOT_STATE_START_MOVE_6);
            break;
        }

        case ROBOT_STATE_START_MOVE_6:
        {
            osDelay(1000);
            /* MOVE_6~MOVE_8阶段y轴切到备用S曲线速度规划参数组(x轴规划器不受影响)、
               y跟踪也切到备用参数组 */
            RobotApplyMove6Params();
            ChassisSetTranslation(&Robot.chassis, 0.38f, -4.6f);
            Robot.state = ROBOT_STATE_WAIT_MOVE_6;
            break;
        }

        case ROBOT_STATE_WAIT_MOVE_6:
        {
            if (ChassisTranslationReached(&Robot.chassis, 2.0f * ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                Robot.state = RobotSkipMove(7, ROBOT_STATE_START_MOVE_7);
            break;
        }

        case ROBOT_STATE_START_MOVE_7:
        {
            osDelay(1000);
            ChassisSetTranslation(&Robot.chassis, 0.38f, -5.79f);
            Robot.state = ROBOT_STATE_WAIT_MOVE_7;
            break;
        }

        case ROBOT_STATE_WAIT_MOVE_7:
        {
            if (ChassisTranslationReached(&Robot.chassis, 2.0f * ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                Robot.state = RobotSkipMove(8, ROBOT_STATE_START_MOVE_8);
            break;
        }

        case ROBOT_STATE_START_MOVE_8:
        {
            osDelay(1000);
            ChassisSetTranslation(&Robot.chassis, 0.38f, -6.98f);
            Robot.state = ROBOT_STATE_WAIT_MOVE_8;
            break;
        }

        case ROBOT_STATE_WAIT_MOVE_8:
        {
            if (ChassisTranslationReached(&Robot.chassis, 2.0f * ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                Robot.state = RobotSkipMove(9, ROBOT_STATE_START_MOVE_9);
            break;
        }

        case ROBOT_STATE_START_MOVE_9:
        {
            /* x、偏航沿用 MOVE_8 已下发的目标继续跟踪(不重新调用
               ChassisSetTranslation/ChassisSetYaw),x跟踪切到第四套参数组,
               仅 y 轴切到固定速度冲坡 */
            osDelay(1000);
            RobotApplyMove9Params();
            ChassisSetRampVelocity(&Robot.chassis, -1.2f);
            Robot.state = ROBOT_STATE_WAIT_MOVE_9;
            break;
        }

        case ROBOT_STATE_WAIT_MOVE_9:
        {
            if (Robot.encoder.y_m <= ROBOT_STATE_MOVE_9_ENCODER_Y_TARGET_M)
            {
                Robot.state = ROBOT_STATE_MANUAL;
            }
            break;
        }

        case ROBOT_STATE_PICKUP:
            Robot.pickup.action = PICKUP_TASK_ACTION_STORE_LOW_HIGH;
            Robot.pickup.active = 1;
            break;

        case ROBOT_STATE_MANUAL:
        {
            ChassisSetVelocity(&Robot.chassis, Robot.zigbee.rx_data.chassis.speed_vx, Robot.zigbee.rx_data.chassis.speed_vy, Robot.zigbee.rx_data.chassis.omega);

            if (Robot.zigbee.rx_data.command.emergency_stop == 1)
            {
                ChassisStop(&Robot.chassis);
            }

            break;
        }

        case ROBOT_STATE_DONE:
        default:
            break;
        }

        osDelay(ROBOT_STATE_UPDATE_PERIOD_MS);
    }
}
