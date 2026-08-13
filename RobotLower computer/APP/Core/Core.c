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

