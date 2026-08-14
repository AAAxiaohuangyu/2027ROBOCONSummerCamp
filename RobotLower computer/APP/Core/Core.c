#include "Core.h"
#include "bsp_config.h"
#include "cmsis_os2.h"

Robot_TypeDef Robot;

/* KFS 1~4 的抓取高度,场地固定为高、低、高、低 */
static const uint8_t kfs_height_table[ROBOT_KFS_COUNT] = {
    PICKUP_HEIGHT_HIGH,
    PICKUP_HEIGHT_LOW,
    PICKUP_HEIGHT_HIGH,
    PICKUP_HEIGHT_LOW,
};

/* 视觉请求/结果的默认占位存储,仅供下面两个弱函数的默认实现使用 */
static uint8_t s_vision_default_index = 1U;

__weak void RobotVisionRequestKfsIndex(uint8_t default_index)
{
    /* 视觉判断留空,记录默认值供下面的默认实现直接采用 */
    s_vision_default_index = default_index;
}

__weak uint8_t RobotVisionKfsIndexReady(uint8_t *result)
{
    /* 视觉判断留空,请求后视为立即完成,直接采用顺序推荐的默认序号 */
    *result = s_vision_default_index;
    return 1U;
}

/* kfs_index为0表示仍在启动区,1~ROBOT_KFS_COUNT为对应KFS工位相对启动区的位置 */
static float RobotKfsPositionX(uint8_t kfs_index)
{
    if (kfs_index == 0U)
    {
        return 0.0f;
    }

    return ROBOT_MOVE_START_TO_KFS1_X + (float)(kfs_index - 1U) * ROBOT_MOVE_KFS_STEP_X;
}

static float RobotKfsPositionY(uint8_t kfs_index)
{
    if (kfs_index == 0U)
    {
        return 0.0f;
    }

    return ROBOT_MOVE_START_TO_KFS1_Y + (float)(kfs_index - 1U) * ROBOT_MOVE_KFS_STEP_Y;
}

/* 下发从kfs_last_index当前所在位置移动到target_index的相对位移 */
static void RobotMoveToKfs(Robot_TypeDef *Robot, uint8_t target_index)
{
    float dx = RobotKfsPositionX(target_index) - RobotKfsPositionX(Robot->pickup.kfs_last_index);
    float dy = RobotKfsPositionY(target_index) - RobotKfsPositionY(Robot->pickup.kfs_last_index);

    ChassisSetTranslation(&Robot->chassis, dx, dy);
}

void RobotInit(void)
{
    RoboticArmInit(&Robot.roboticarm,
                   ROBOTICARM_LIFT_FDCAN_HANDLE, ROBOTICARM_LIFT_ID,
                   ROBOTICARM_FORWARD_UART_HANDLE, ROBOTICARM_FORWARD_ID,
                   ROBOTICARM_ROTATE_UART_HANDLE, ROBOTICARM_ROTATE_ID);

    ChassisInit(&Robot.chassis, CHASSIS_FDCAN_HANDLE, CHASSIS_CTRL_ID);

    /* zigbee.c头部注释要求Zigbee_Init须在MX_USART1_UART_Init()之后调用,以开启huart1的
       DMA接收(HAL_UARTEx_ReceiveToIdle_DMA);huart1由CubeMX固定分配,非占位句柄,无需判空 */
    Zigbee_Init(&Robot.zigbee);

    /* 各外设句柄在CubeMX完成分配前于bsp_config.h中为NULL占位,逐个判空后再启动,
       句柄补齐后无需再改这里 */
    if (Robot.roboticarm.lift_motor.FDCAN_Handle != NULL)
    {
        uint16_t lift_feedback_base = (uint16_t)((J60_RESPONSE_FEEDBACK << J60_CAN_ID_RESPONSE_SHIFT) |
                                                 (J60_CMD_CONTROL << J60_CAN_ID_COMMAND_SHIFT));
        FDCANStandardInit(Robot.roboticarm.lift_motor.FDCAN_Handle,
                          lift_feedback_base + J60_ID_MIN,
                          lift_feedback_base + J60_ID_MAX);
    }

    if (Robot.chassis.drive.motor_group.FDCAN_Handle != NULL)
    {
        FDCANStandardInit(Robot.chassis.drive.motor_group.FDCAN_Handle,
                          M3508_FEEDBACK_ID_BASE + M3508_ID_MIN,
                          M3508_FEEDBACK_ID_BASE + M3508_ID_MAX);
    }

    if (Robot.roboticarm.forward_motor.huart != NULL)
    {
        HAL_UARTEx_ReceiveToIdle_IT(Robot.roboticarm.forward_motor.huart,
                                    Robot.roboticarm.forward_motor.feedback.packet.bytes,
                                    GO_M8010_FEEDBACK_FRAME_SIZE);
    }

    if (Robot.roboticarm.rotate_motor.huart != NULL)
    {
        HAL_UARTEx_ReceiveToIdle_IT(Robot.roboticarm.rotate_motor.huart,
                                    Robot.roboticarm.rotate_motor.feedback.packet.bytes,
                                    GO_M8010_FEEDBACK_FRAME_SIZE);
    }

    RoboticArmEnable(&Robot.roboticarm);
}

void RobotStateUpdate(Robot_TypeDef *Robot)
{
    while (1)
    {
        switch (Robot->state)
        {
        case ROBOT_STATE_IDLE:
            Robot->pickup.kfs_last_index = 0U;
            Robot->pickup.kfs_target_index = 0U;
            Robot->pickup.vision_request_sent = 0U;
            RoboticArmEnable(&Robot->roboticarm);
            Robot->state = ROBOT_STATE_MOVE1;
            break;

        case ROBOT_STATE_MOVE1:
            /* 从启动区运动到翻转区 */
            ChassisSetTranslation(&Robot->chassis, ROBOT_MOVE_START_TO_FLIP_X, ROBOT_MOVE_START_TO_FLIP_Y);
            if (ChassisTranslationReached(&Robot->chassis, ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                Robot->state = ROBOT_STATE_FLIP;
            break;

        case ROBOT_STATE_FLIP:
            /* 翻转KFS */
            RoboticArmFlipMotion(&Robot->roboticarm, &Robot->flip_state);
            if (Robot->flip_state == FLIP_STATE_DONE)
                Robot->state = ROBOT_STATE_MOVE2;
            break;

        case ROBOT_STATE_MOVE2:
            /* 从翻转区返回启动区 */
            ChassisSetTranslation(&Robot->chassis, ROBOT_MOVE_FLIP_TO_START_X, ROBOT_MOVE_FLIP_TO_START_Y);
            if (ChassisTranslationReached(&Robot->chassis, ROBOT_CHASSIS_POSITION_TOLERANCE_M))
            {
                Robot->pickup.vision_next_state = ROBOT_STATE_MOVE3;
                Robot->pickup.vision_request_sent = 0U;
                Robot->state = ROBOT_STATE_VISION_WAIT;
            }
            break;

        case ROBOT_STATE_VISION_WAIT:
            /* 等待视觉判断下一个KFS是否需要抓取,判断结果就绪前底盘/机械臂均保持当前状态不动 */
            if (!Robot->pickup.vision_request_sent)
            {
                uint8_t default_index = (Robot->pickup.vision_next_state == ROBOT_STATE_MOVE3) ? 1U : (uint8_t)(Robot->pickup.kfs_last_index + 1U);
                RobotVisionRequestKfsIndex(default_index);
                Robot->pickup.vision_request_sent = 1U;
            }
            else
            {
                uint8_t result;
                if (RobotVisionKfsIndexReady(&result))
                {
                    Robot->pickup.kfs_target_index = result;
                    Robot->state = Robot->pickup.vision_next_state;
                }
            }
            break;

        case ROBOT_STATE_MOVE3:
        case ROBOT_STATE_MOVE4:
        case ROBOT_STATE_MOVE5:
            /* 运动到当前目标KFS工位,到位后执行PICKUP,完成后返回的状态按MOVE3/4/5依次递进 */
            RobotMoveToKfs(Robot, Robot->pickup.kfs_target_index);
            if (ChassisTranslationReached(&Robot->chassis, ROBOT_CHASSIS_POSITION_TOLERANCE_M))
            {
                Robot->pickup.pickup_return_state = (Robot->state == ROBOT_STATE_MOVE3) ? ROBOT_STATE_MOVE4 : (Robot->state == ROBOT_STATE_MOVE4) ? ROBOT_STATE_MOVE5
                                                                                                                                           : ROBOT_STATE_MOVE6;
                Robot->pickup.pick_state = PICKUP_STATE_RAISE;
                Robot->state = ROBOT_STATE_PICKUP;
            }
            break;

        case ROBOT_STATE_PICKUP:
            /* 拾取当前目标KFS,吸取高度由KFS序号(1~4:高低高低)查表决定;
               本轮第1/2/3次拾取动作固定为存底层/存上层/保持真空(由pickup_return_state区分次序) */
            if (Robot->pickup.pickup_return_state == ROBOT_STATE_MOVE4)
            {
                RoboticArmPickupStoreLowMotion(&Robot->roboticarm, &Robot->pickup.pick_state,
                                               kfs_height_table[Robot->pickup.kfs_target_index - 1U]);
            }
            else if (Robot->pickup.pickup_return_state == ROBOT_STATE_MOVE5)
            {
                RoboticArmPickupStoreHighMotion(&Robot->roboticarm, &Robot->pickup.pick_state,
                                                kfs_height_table[Robot->pickup.kfs_target_index - 1U]);
            }
            else
            {
                RoboticArmPickupHoldMotion(&Robot->roboticarm, &Robot->pickup.pick_state,
                                           kfs_height_table[Robot->pickup.kfs_target_index - 1U]);
            }

            /* StoreLow/StoreHigh完成后回到VOID;Hold完成后停在HOLD,同样视为本次拾取完成 */
            if (Robot->pickup.pick_state == PICKUP_STATE_VOID || Robot->pickup.pick_state == PICKUP_STATE_HOLD)
            {
                Robot->pickup.kfs_last_index = Robot->pickup.kfs_target_index;
                if (Robot->pickup.pickup_return_state != ROBOT_STATE_MOVE6)
                {
                    Robot->pickup.vision_next_state = Robot->pickup.pickup_return_state;
                    Robot->pickup.vision_request_sent = 0U;
                    Robot->state = ROBOT_STATE_VISION_WAIT;
                }
                else
                {
                    Robot->state = Robot->pickup.pickup_return_state;
                }
            }
            break;

        case ROBOT_STATE_MOVE6:
            /* 最后一个正确的KFS拾取完成,上斜坡 */
            ChassisSetTranslation(&Robot->chassis, ROBOT_MOVE_UP_SLOPE_X, ROBOT_MOVE_UP_SLOPE_Y);
            if (ChassisTranslationReached(&Robot->chassis, ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                Robot->state = ROBOT_STATE_MANUAL;
            break;

        case ROBOT_STATE_MANUAL:
            /* 手动操作模式,控制逻辑由其他模块负责,此处不作处理 */
            break;

        default:
            Robot->state = ROBOT_STATE_IDLE;
            break;
        }

        osDelay(ROBOT_STATE_UPDATE_PERIOD_MS);
    }
}
