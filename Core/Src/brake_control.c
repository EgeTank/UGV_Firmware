//Elektromanyetik fren sistemi için yazılım.

#include "brake_control.h"
#include "stm32h7xx_hal.h"
#include "system_config.h"

/* Fren varsayılan: KAPALI */
static volatile uint8_t brake_applied = 1;

void brake_init(void)
{
    brake_applied = 1;
    HAL_GPIO_WritePin(BRAKE_GPIO_PORT, BRAKE_PIN, GPIO_PIN_RESET);
}

void brake_apply(void)
{
    brake_applied = 1;
    HAL_GPIO_WritePin(BRAKE_GPIO_PORT, BRAKE_PIN, GPIO_PIN_RESET);
}

void brake_release(void)
{
    brake_applied = 0;
    HAL_GPIO_WritePin(BRAKE_GPIO_PORT, BRAKE_PIN, GPIO_PIN_SET);
}

uint8_t brake_is_applied(void)
{
    return brake_applied;
}
