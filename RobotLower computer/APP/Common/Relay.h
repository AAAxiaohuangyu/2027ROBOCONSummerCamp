#ifndef __RELAY_H__
#define __RELAY_H__

#include "stm32h7xx_hal.h"

/* PB3 继电器控制：高电平通电吸合，低电平断电释放。 */
#define RELAY_GPIO_PORT GPIOB
#define RELAY_GPIO_PIN  GPIO_PIN_3
#define RELAY_ON        GPIO_PIN_SET
#define RELAY_OFF       GPIO_PIN_RESET

HAL_StatusTypeDef RelayInit(void);
void RelayOn(void);
void RelayOff(void);
GPIO_PinState RelayGetState(void);

#endif /* __RELAY_H__ */
