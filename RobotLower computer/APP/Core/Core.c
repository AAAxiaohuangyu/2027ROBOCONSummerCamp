#include "Core.h"
#include "bsp_config.h"

Robot_TypeDef Robot;

void RobotInit(void)
{
    RoboticArmInit(&Robot.roboticarm,
                    ROBOTICARM_LIFT_FDCAN_HANDLE, ROBOTICARM_LIFT_ID,
                    ROBOTICARM_FORWARD_UART_HANDLE, ROBOTICARM_FORWARD_ID,
                    ROBOTICARM_ROTATE_UART_HANDLE, ROBOTICARM_ROTATE_ID);

    /* TODO: 底盘(Chassis)等其余子系统init待补充 */
}

void RobotStateUpdate(void)
{
    /* TODO: 占位,后续补充 */
}

