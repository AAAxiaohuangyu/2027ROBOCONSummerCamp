#include "Core.h"
#include "bsp_config.h"

Robot_TypeDef Robot;

void RobotInit(void)
{
    RoboticArmInit(&Robot.roboticarm,
                    ROBOTICARM_LIFT_FDCAN_HANDLE, ROBOTICARM_LIFT_ID,
                    ROBOTICARM_FORWARD_UART_HANDLE, ROBOTICARM_FORWARD_ID,
                    ROBOTICARM_ROTATE_UART_HANDLE, ROBOTICARM_ROTATE_ID);

    ChassisInit(&Robot.chassis, CHASSIS_FDCAN_HANDLE, CHASSIS_CTRL_ID);

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
}

void RobotStateUpdate(Robot_TypeDef *Robot)
{
    while (1)
    {
        switch (Robot->flip_state)
        {
        case ROBOT_STATE_IDLE:
            /* code */
            break;
        
        default:
            break;
        }
    }
}

