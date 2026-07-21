#include "motor_control.h"
#include "system_config.h"
#include "stm32h7xx_hal.h"
#include "safety.h"
#include "odrive_can.h"

/* CubeMX'in (tim.c veya main.c) ürettiği değişkene bağlanıyoruz */
extern TIM_HandleTypeDef htim1;

static float target_speed_mm_s = 0.0f;

void motor_init(void)
{
    /* Init işlemi zaten main() içinde CubeMX tarafından yapıldı!
       Biz burada sadece PWM'i başlatıyoruz. */
    HAL_TIM_PWM_Start(&htim1, MOTOR_PWM_CHANNEL);
    odrive_init();
}

static uint32_t speed_to_duty_float(float speed_m_s)
{
    const float MAX_SPEED_M_S = 3.0f;

    float abs_speed = (speed_m_s >= 0.0f) ? speed_m_s : -speed_m_s;
    if (abs_speed > MAX_SPEED_M_S)
        abs_speed = MAX_SPEED_M_S;

    uint32_t duty =
        (uint32_t)((abs_speed / MAX_SPEED_M_S) * (float)MAX_DUTY_CYCLE);

    if (duty > MAX_DUTY_CYCLE)
        duty = MAX_DUTY_CYCLE;

    return duty;
}

void motor_apply_drive_new(const command_t *cmd)
{
    if (Safety_Check() != SAFETY_OK)
    {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_CHANNEL, 0);
        return;
    }

    if (cmd->frenle > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_CHANNEL, 0);
        return;
    }

    uint32_t duty = speed_to_duty_float(cmd->hizlan);
    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_CHANNEL, duty);
}

int read_safety_status(void)
{
    return (Safety_Check() == SAFETY_OK) ? 0 : 1;
}

void motor_apply_fail_safe(uint8_t brake_active)
{
    if (brake_active)
    {
        __HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_CHANNEL, 0);
    }
}

void motor_set_target_speed(float speed_mm_s)
{
    target_speed_mm_s = speed_mm_s;
}

float motor_get_target_speed(void)
{
    return target_speed_mm_s;
}

void motor_set_output(float pid_output)
{
    const float MAX_PID = 100.0f;

    /* 1. PWM (Fren veya eski donanım için) Hesaplamaları */
    float abs_output = (pid_output >= 0.0f) ? pid_output : -pid_output;
    if (abs_output > MAX_PID) abs_output = MAX_PID;

    uint32_t duty = (uint32_t)((abs_output / MAX_PID) * (float)MAX_DUTY_CYCLE);
    if (duty > MAX_DUTY_CYCLE) duty = MAX_DUTY_CYCLE;

    __HAL_TIM_SET_COMPARE(&htim1, MOTOR_PWM_CHANNEL, duty);

    /* 2. ODrive CAN FD Komutu - DURUM KONTROLLÜ */

    const odrive_state_t *odrv = odrive_get_state();

    if (odrv != NULL && odrv->heartbeat.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP)
    {
        const float MAX_SPEED_MPS = 3.0f;
        float speed_mps = (pid_output / MAX_PID) * MAX_SPEED_MPS;
        float turns_per_s = odrive_mps_to_turns_per_s(speed_mps);

        odrive_send_velocity(turns_per_s, 0.0f);
    }
}
//Odrive kontrolcüsü CLOSED_MODE olarak ayarlandı bu sayede kalkışta jerk tamamen önlenecek.
//Masa başı testleri yaparken bazı veriler görünmeyebilir.
/*
 * motor_set_output fonksiyonunu akıllandırdık. Eskiden araç ODrive IDLE (boşta) modundayken bile sürekli hız komutu yollamaya çalışıyordu.

Artık sadece CLOSED_LOOP modundayken komut gönderiyoruz. Bu sayede hem 8 Mbps CAN hattındaki gereksiz çöp trafiği temizledik hem de aracın mod geçişlerinde hedef hızı biriktirip bir anda mekanik aksama vuruntu (jerk) yapmasını, yani ani şahlanmaları engelledik.*/
