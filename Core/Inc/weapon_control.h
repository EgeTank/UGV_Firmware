#ifndef WEAPON_CONTROL_H
#define WEAPON_CONTROL_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

/* Donanım Pin Tanımlamaları - CubeMX'ten ayarlandığı varsayılmıştır */
#define LASER_GPIO_PORT     GPIOC
#define LASER_PIN           GPIO_PIN_13
#define FIRE_RELAY_PORT     GPIOC
#define FIRE_RELAY_PIN      GPIO_PIN_14

/* Atış rölesinin maksimum açık kalma süresi (Solenoid koruması) */
#define FIRE_MAX_DURATION_MS 200U

void weapon_init(void);
void weapon_laser_set(uint8_t state);
void weapon_fire_trigger_isr(void);
void weapon_task_check(void);

#endif /* WEAPON_CONTROL_H */
