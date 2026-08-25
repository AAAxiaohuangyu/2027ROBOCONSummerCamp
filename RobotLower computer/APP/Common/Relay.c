#include "Relay.h"

static GPIO_PinState relay_state = RELAY_OFF;

HAL_StatusTypeDef RelayInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = RELAY_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RELAY_GPIO_PORT, &gpio);

    RelayOff();
    return HAL_OK;
}

void RelayOn(void)
{
    relay_state = RELAY_ON;
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, relay_state);
}

void RelayOff(void)
{
    relay_state = RELAY_OFF;
    HAL_GPIO_WritePin(RELAY_GPIO_PORT, RELAY_GPIO_PIN, relay_state);
}

GPIO_PinState RelayGetState(void)
{
    return relay_state;
}
