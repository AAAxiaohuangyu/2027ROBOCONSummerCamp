#include "GasPump.h"

/* 机械臂夹取动作 - 打开气泵 */
void RoboticArmGripMotion(void)
{
    HAL_GPIO_WritePin(GASPUMP_GPIO_PORT, GASPUMP_GPIO_PIN, GASPUMP_ON);
}

/* 机械臂释放动作 - 关闭气泵 */
void RoboticArmReleaseMotion(void)
{
    HAL_GPIO_WritePin(GASPUMP_GPIO_PORT, GASPUMP_GPIO_PIN, GASPUMP_OFF);
}
