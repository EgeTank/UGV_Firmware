#include "state_machine.h"
#include "failsafe.h"
#include "cmsis_os2.h"

extern osMutexId_t stateMutexHandle;   // freertos.c'de tanımlı
extern vehicle_state_t g_vehicleState; // freertos.c'de tanımlı

void state_machine_init(void)
{
    g_vehicleState = STATE_STANDBY;
}

vehicle_state_t state_machine_get(void)
{
    vehicle_state_t state;
    if (osMutexAcquire(stateMutexHandle, osWaitForever) == osOK) {
        state = g_vehicleState;
        osMutexRelease(stateMutexHandle);
    } else {
        state = STATE_EMERGENCY;
    }
    return state;
}

void state_machine_set(vehicle_state_t new_state)
{
    if (osMutexAcquire(stateMutexHandle, osWaitForever) != osOK) return;

    if (failsafe_is_active() && new_state == STATE_DRIVING) {
        g_vehicleState = STATE_EMERGENCY;
    } else {
        g_vehicleState = new_state;
    }
    osMutexRelease(stateMutexHandle);
}

// NOT: state_machine_update() tamamen SİLİNDİ.
// Çünkü motor_set_output ve brake_apply işlemleri ControlTask içinde yapılıyor.
