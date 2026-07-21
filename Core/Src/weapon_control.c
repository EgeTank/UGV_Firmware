#include "weapon_control.h"
#include "failsafe.h"


static volatile uint32_t fire_start_time = 0;
static volatile uint8_t  is_firing = 0;

void weapon_init(void)
{
    /* * NOT: PC13 ve PC14 pinlerinin donanımsal kurulumu (Output_PP vb.)
     * zaten gpio.c içindeki MX_GPIO_Init() fonksiyonunda yapıldı.
     */

    /* 1. Sistem açılırken röle yanlışlıkla tetiklenmesin diye pinleri güvenli seviyeye (LOW) çek */
    HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(FIRE_RELAY_PORT, FIRE_RELAY_PIN, GPIO_PIN_RESET);

    /* 2. Mantıksal durum değişkenini sıfırla */
    is_firing = 0;
}
void weapon_laser_set(uint8_t state)
{
    if(state) {
        HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_PIN, GPIO_PIN_RESET);
    }
}

/* EXTI'dan çağrılacak asıl kesme rutini (Zero-latency) */
void weapon_fire_trigger_isr(void)
{
    if(!is_firing && !failsafe_is_active()) {
        HAL_GPIO_WritePin(FIRE_RELAY_PORT, FIRE_RELAY_PIN, GPIO_PIN_SET);
        fire_start_time = HAL_GetTick();
        is_firing = 1;
    }
}

/* Ana döngüde (ControlTask) çağrılıp röleyi güvenli şekilde kapatır */
void weapon_task_check(void)
{
    if (is_firing) {
        if ((HAL_GetTick() - fire_start_time) > FIRE_MAX_DURATION_MS) {
            HAL_GPIO_WritePin(FIRE_RELAY_PORT, FIRE_RELAY_PIN, GPIO_PIN_RESET);
            is_firing = 0;
        }
    }
}

