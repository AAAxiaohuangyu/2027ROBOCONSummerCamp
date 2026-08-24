#include "Core.h"
#include "bsp_config.h"
#include "cmsis_os2.h"
#include "GasPump.h"

Robot_TypeDef Robot;

/* KFS 1~4 的抓取高度,场地固定为高、低、高、低 */
static const uint8_t kfs_height_table[ROBOT_KFS_COUNT] = {
    PICKUP_HEIGHT_HIGH,
    PICKUP_HEIGHT_LOW,
    PICKUP_HEIGHT_HIGH,
    PICKUP_HEIGHT_LOW,
};

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

/* 下发从当前位置移动到target_index对应KFS工位的绝对目标坐标 */
static void RobotMoveToKfs(Robot_TypeDef *Robot, uint8_t target_index)
{
    ChassisSetTranslation(&Robot->chassis, RobotKfsPositionX(target_index), RobotKfsPositionY(target_index));
}

void RobotInit(void)
{
    /* RobotInit在调度器启动前单线程执行,不存在并发访问,可安全去除Robot的volatile限定 */
    Robot_TypeDef *robot = (Robot_TypeDef *)&Robot;

    RoboticArmInit(&robot->roboticarm,
                   ROBOTICARM_LIFT_FDCAN_HANDLE, ROBOTICARM_LIFT_ID,
                   ROBOTICARM_FORWARD_UART_HANDLE, ROBOTICARM_FORWARD_ID,
                   ROBOTICARM_ROTATE_UART_HANDLE, ROBOTICARM_ROTATE_ID);

    ChassisInit(&robot->chassis, CHASSIS_FDCAN_HANDLE, CHASSIS_CTRL_ID);

    /* zigbee.c头部注释要求Zigbee_Init须在MX_USART1_UART_Init()之后调用,以开启huart1的
       DMA接收(HAL_UARTEx_ReceiveToIdle_DMA);huart1由CubeMX固定分配,非占位句柄,无需判空 */
    Zigbee_Init(&robot->zigbee);

    /* VISION_UART_HANDLE在CubeMX完成分配前于bsp_config.h中为NULL占位,Vision_Init内部
       自行判空,无需在此额外判断 */
    Vision_Init(&robot->vision);

    /* 各外设句柄在CubeMX完成分配前于bsp_config.h中为NULL占位,逐个判空后再启动,
       句柄补齐后无需再改这里 */
    if (robot->roboticarm.lift_motor.FDCAN_Handle != NULL)
    {
        uint16_t lift_feedback_base = (uint16_t)((J60_RESPONSE_FEEDBACK << J60_CAN_ID_RESPONSE_SHIFT) |
                                                 (J60_CMD_CONTROL << J60_CAN_ID_COMMAND_SHIFT));
        FDCANStandardInit(robot->roboticarm.lift_motor.FDCAN_Handle,
                          lift_feedback_base + J60_ID_MIN,
                          lift_feedback_base + J60_ID_MAX);
    }

    if (robot->chassis.drive.motor_group.FDCAN_Handle != NULL)
    {
        FDCANStandardInit(robot->chassis.drive.motor_group.FDCAN_Handle,
                          M3508_FEEDBACK_ID_BASE + M3508_ID_MIN,
                          M3508_FEEDBACK_ID_BASE + M3508_ID_MAX);
    }

    /* forward/rotate两个GO电机的接收挂起改由GOM8010GroupUpdate在每次真正发起请求时按需完成
       (RS485总线仲裁,避免同一huart被同时挂起两次接收),此处不再手动挂起 */

    RoboticArmEnable(&robot->roboticarm);
}

/*void RobotStateUpdate(Robot_TypeDef *Robot)
{
    while (1)
    {
        switch (Robot->state)
        {
        case ROBOT_STATE_IDLE:
            Robot->pickup.kfs_last_index = 0U;
            Robot->pickup.kfs_target_index = 0U;
            RoboticArmEnable(&Robot->roboticarm);
            Robot->state = ROBOT_STATE_MOVE1;
            break;

        case ROBOT_STATE_MOVE1:
            ChassisSetTranslation(&Robot->chassis, ROBOT_MOVE_START_TO_FLIP_X, ROBOT_MOVE_START_TO_FLIP_Y);
            if (ChassisTranslationReached(&Robot->chassis, ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                Robot->state = ROBOT_STATE_FLIP;
            break;

        case ROBOT_STATE_FLIP:
            RoboticArmFlipMotion(&Robot->roboticarm, &Robot->flip_state);
            if (Robot->flip_state == FLIP_STATE_DONE)
                Robot->state = ROBOT_STATE_MOVE2;
            break;

        case ROBOT_STATE_MOVE2:
            ChassisSetTranslation(&Robot->chassis, ROBOT_MOVE_FLIP_TO_START_X, ROBOT_MOVE_FLIP_TO_START_Y);
            if (ChassisTranslationReached(&Robot->chassis, ROBOT_CHASSIS_POSITION_TOLERANCE_M))
            {
                Robot->pickup.vision_next_state = ROBOT_STATE_MOVE3;
                Robot->state = ROBOT_STATE_VISION_WAIT;
            }
            break;

        case ROBOT_STATE_VISION_WAIT:
            if (Robot->vision.next_colour != KFS_COLOUR_UNKNOWN)
            {
                uint8_t default_index = (Robot->pickup.vision_next_state == ROBOT_STATE_MOVE3) ? 1U : (uint8_t)(Robot->pickup.kfs_last_index + 1U);;
                Robot->pickup.kfs_target_index = (Robot->vision.next_colour == VISION_CORRECT_COLOUR) ? default_index : (uint8_t)(default_index + 1U);
                Robot->state = Robot->pickup.vision_next_state;
            }
            break;

        case ROBOT_STATE_MOVE3:
        case ROBOT_STATE_MOVE4:
        case ROBOT_STATE_MOVE5:
    
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

            if (Robot->pickup.pick_state == PICKUP_STATE_VOID || Robot->pickup.pick_state == PICKUP_STATE_HOLD)
            {
                Robot->pickup.kfs_last_index = Robot->pickup.kfs_target_index;
                if (Robot->pickup.pickup_return_state != ROBOT_STATE_MOVE6)
                {
                    Robot->pickup.vision_next_state = Robot->pickup.pickup_return_state;
                    Robot->state = ROBOT_STATE_VISION_WAIT;
                }
                else
                {
                    Robot->state = Robot->pickup.pickup_return_state;
                }
            }
            break;

        case ROBOT_STATE_MOVE6:
            ChassisSetTranslation(&Robot->chassis, ROBOT_MOVE_UP_SLOPE_X, ROBOT_MOVE_UP_SLOPE_Y);
            if (ChassisTranslationReached(&Robot->chassis, ROBOT_CHASSIS_POSITION_TOLERANCE_M))
                Robot->state = ROBOT_STATE_MANUAL;
            break;

        case ROBOT_STATE_MANUAL:
        {
            float lift_velocity = 0.0f;
            float forward_velocity = 0.0f;
            float rotate_velocity = 0.0f;

          
            ChassisSetVelocity(&Robot->chassis, (float)Robot->zigbee.explained_data.chassis.speed_vx,
                               (float)Robot->zigbee.explained_data.chassis.speed_vy,
                               (float)Robot->zigbee.explained_data.chassis.omega);

            if (Robot->zigbee.explained_data.joint.up_down == 1)
                lift_velocity = ROBOT_MANUAL_ARM_VELOCITY;
            else if (Robot->zigbee.explained_data.joint.up_down == 2)
                lift_velocity = -ROBOT_MANUAL_ARM_VELOCITY;

            if (Robot->zigbee.explained_data.joint.front_back == 1)
                forward_velocity = ROBOT_MANUAL_ARM_VELOCITY;
            else if (Robot->zigbee.explained_data.joint.front_back == 2)
                forward_velocity = -ROBOT_MANUAL_ARM_VELOCITY;

            if (Robot->zigbee.explained_data.joint.flip == 1)
                rotate_velocity = ROBOT_MANUAL_ARM_VELOCITY;
            else if (Robot->zigbee.explained_data.joint.flip == 2)
                rotate_velocity = -ROBOT_MANUAL_ARM_VELOCITY;

            J60MotorSetVelocityTarget(&Robot->roboticarm.lift_motor, lift_velocity);
            GOM8010GroupSetVelocityTarget(&Robot->roboticarm.go_motors, ROBOTICARM_GO_FORWARD, forward_velocity);
            GOM8010GroupSetVelocityTarget(&Robot->roboticarm.go_motors, ROBOTICARM_GO_ROTATE, rotate_velocity);

          
            if (Robot->zigbee.explained_data.command.emergency_stop != 0U)
                RoboticArmReleaseMotion();
            else if (Robot->zigbee.explained_data.command.grab != 0U)
                RoboticArmGripMotion();
            else
                RoboticArmReleaseMotion();

            break;
        }
        default:
            Robot->state = ROBOT_STATE_IDLE;
            break;
        }

        osDelay(ROBOT_STATE_UPDATE_PERIOD_MS);
    }
}
*/
