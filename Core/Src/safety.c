#include "safety.h"
#include "system_config.h"
#include "stm32h7xx_hal.h"
#include "failsafe.h"

void Safety_Init(void)
{
    // GPIO init CubeMX tarafında varsayılıyor
}

safety_status_t Safety_Check(void)
{
    if (HAL_GPIO_ReadPin(ESTOP_GPIO_PORT, ESTOP_PIN) == GPIO_PIN_RESET)
        return SAFETY_ESTOP_ACTIVE;

    if (HAL_GPIO_ReadPin(BMS_GPIO_PORT, BMS_PIN) == GPIO_PIN_RESET)
        return SAFETY_BMS_FAULT;

    return SAFETY_OK;
}

/* * EXTI Callback Fonksiyonu: E-STOP pininde gerilim düştüğü an
 * HAL_GPIO_EXTI_IRQHandler tarafından anında burası çağrılır.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ESTOP_PIN)
    {
        /*
         * RTOS task'lerini veya ana döngüyü beklemez.
         * Anında EV200 kontaktörünü açar (48V kesilir),
         * motorlara giden PWM'i sıfırlar ve frenleri kilitler.
         */
        failsafe_force();
    }
}
