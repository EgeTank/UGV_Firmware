#ifndef CURRENT_LIMITER_H
#define CURRENT_LIMITER_H

#include <stdint.h>
#include "system_config.h"   /* CURRENT_LIM_* sabitleri buradan gelir */

typedef struct {
    float nominal_limit_a;
    float peak_limit_a;
    float min_limit_a;

    float bus_voltage_v;
    float motor_temp_c;
    float measured_current_a;

    float current_limit_a;

    float softstart_elapsed_ms;
    uint8_t softstart_done;

    uint32_t limit_hit_count;

    /* Limp mode veya acil durumlar için override flag */
    uint8_t is_override_active;
} current_limiter_t;

void     current_limiter_init(current_limiter_t *c, float nominal_a, float peak_a, float min_a);
void     current_limiter_update(current_limiter_t *c, float voltage_v, float temp_c, float current_a, float dt_ms);
float    current_limiter_apply(current_limiter_t *c, float pid_output);

/* Override yönetimi */
void     current_limiter_set_override(current_limiter_t *c, uint8_t active);

float    current_limiter_get_limit(const current_limiter_t *c);
uint32_t current_limiter_get_hit_count(const current_limiter_t *c);
void     current_limiter_reset(current_limiter_t *c);

#endif /* CURRENT_LIMITER_H */
