#include "GasPump.h"

/* 机械臂夹取动作 - 打开气泵 */
void RoboticArmGripMotion(void)
{
    /* 这里只输出抓取控制信号；是否已建立真空需要由硬件或后续传感器策略确认。 */
    HAL_GPIO_WritePin(GASPUMP_GPIO_PORT, GASPUMP_GPIO_PIN, GASPUMP_ON);
}

/* 机械臂释放动作 - 关闭气泵 */
void RoboticArmReleaseMotion(void)
{
    /* 这里只输出释放控制信号；不能据此直接推断工件已经从吸盘脱离。 */
    HAL_GPIO_WritePin(GASPUMP_GPIO_PORT, GASPUMP_GPIO_PIN, GASPUMP_OFF);
}
