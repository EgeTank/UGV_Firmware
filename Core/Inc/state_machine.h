#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

/*
   ARAÇ DURUMLARI (ENUM)
    */
typedef enum
{
    STATE_STANDBY = 0,   // Beklemede (motor kapalı, frenli)
    STATE_DRIVING,       // Seyir (PID aktif)
    STATE_EMERGENCY      // Acil durum (fail-safe)
} vehicle_state_t;


void state_machine_init(void);
void state_machine_update(void);

vehicle_state_t state_machine_get(void);
void state_machine_set(vehicle_state_t new_state);

#endif
