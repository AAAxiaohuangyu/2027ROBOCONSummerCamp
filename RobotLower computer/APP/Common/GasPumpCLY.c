#include "GasPumpCLY.h"

void GasPumpOn(void)
{
    HAL_GPIO_WritePin(GASPUMP_GPIO_PORT, GASPUMP_GPIO_PIN, GASPUMP_ON);
}

void GasPumpOff(void)
{
    HAL_GPIO_WritePin(GASPUMP_GPIO_PORT, GASPUMP_GPIO_PIN, GASPUMP_OFF);
}

//吸盘吸住
void CLY_On(void)
{
    HAL_GPIO_WritePin(CLY_GPIO_PORT, CLY_GPIO_PIN, CLY_ON);
}

//吸盘释放
void CLY_Off(void)
{
    HAL_GPIO_WritePin(CLY_GPIO_PORT, GASPUMP_GPIO_PIN, CLY_OFF);
}





