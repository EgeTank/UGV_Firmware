#ifndef ODRIVE_SM_H
#define ODRIVE_SM_H

#include <stdint.h>
#include "odrive_can.h"
#include "system_config.h"   /* ODRIVE_SM_TIMEOUT_MS buradan gelir */
#include "state_machine.h"

typedef enum {
    ODRIVE_SM_IDLE = 0,
    ODRIVE_SM_REQUESTING_CLOSED_LOOP,
    ODRIVE_SM_CLOSED_LOOP,
    ODRIVE_SM_REQUESTING_IDLE,
    ODRIVE_SM_ERROR
} odrive_sm_state_t;

typedef struct {
    odrive_sm_state_t state;
    uint32_t          request_tick;
    uint32_t          timeout_ms;
    uint32_t          error_count;
} odrive_sm_t;

void     odrive_sm_init(odrive_sm_t *sm);
void odrive_sm_update(odrive_sm_t *sm, vehicle_state_t vehicle_state);uint8_t  odrive_sm_is_closed_loop(const odrive_sm_t *sm);
uint32_t odrive_sm_get_error_count(const odrive_sm_t *sm);
void     odrive_sm_force_idle(odrive_sm_t *sm);

#endif /* ODRIVE_SM_H */
