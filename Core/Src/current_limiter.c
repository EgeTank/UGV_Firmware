#include "current_limiter.h"

/* ============================================================
   YARDIMCI: clamp
   ============================================================ */
static float clamp(float val, float lo, float hi)
{
    if (val > hi) return hi;
    if (val < lo) return lo;
    return val;
}

/* ============================================================
   YARDIMCI: lineer interpolasyon
   ============================================================ */
static float lerp(float x, float x0, float x1, float y0, float y1)
{
    if (x <= x0) return y0;
    if (x >= x1) return y1;
    return y0 + (y1 - y0) * ((x - x0) / (x1 - x0));
}

/* ============================================================
   current_limiter_init()
   ============================================================ */
void current_limiter_init(current_limiter_t *c,
                          float nominal_a,
                          float peak_a,
                          float min_a)
{
    c->nominal_limit_a      = nominal_a;
    c->peak_limit_a         = peak_a;
    c->min_limit_a          = min_a;
    c->current_limit_a      = 0.0f;
    c->bus_voltage_v        = CURRENT_LIM_VOLTAGE_NOMINAL;
    c->motor_temp_c         = 25.0f;
    c->measured_current_a   = 0.0f;
    c->softstart_elapsed_ms = 0.0f;
    c->softstart_done       = 0U;
    c->limit_hit_count      = 0U;
    c->is_override_active   = 0U; /* Başlangıçta normal mod */
}

/* ============================================================
   current_limiter_set_override()
   ============================================================ */
void current_limiter_set_override(current_limiter_t *c, uint8_t active)
{
    c->is_override_active = active;
}

/* ============================================================
   current_limiter_reset()
   ============================================================ */
void current_limiter_reset(current_limiter_t *c)
{
    c->current_limit_a      = 0.0f;
    c->softstart_elapsed_ms = 0.0f;
    c->softstart_done       = 0U;
    c->limit_hit_count      = 0U;
    c->is_override_active   = 0U;
}

void current_limiter_update(current_limiter_t *c,
                            float voltage_v,
                            float temp_c,
                            float current_a,
                            float dt_ms)
{
    c->bus_voltage_v      = voltage_v;
    c->motor_temp_c       = temp_c;
    c->measured_current_a = current_a;

    float softstart_limit;
    if (!c->softstart_done)
    {
        c->softstart_elapsed_ms += dt_ms;
        if (c->softstart_elapsed_ms >= (float)CURRENT_LIM_SOFTSTART_MS)
        {
            c->softstart_done = 1U;
            softstart_limit   = c->nominal_limit_a;
        }
        else
        {
            softstart_limit = lerp(c->softstart_elapsed_ms,
                                   0.0f,
                                   (float)CURRENT_LIM_SOFTSTART_MS,
                                   0.0f,
                                   c->nominal_limit_a);
        }
    }
    else
    {
        softstart_limit = c->nominal_limit_a;
    }

    float voltage_factor = lerp(voltage_v,
                                CURRENT_LIM_VOLTAGE_MIN,
                                CURRENT_LIM_VOLTAGE_NOMINAL,
                                0.5f,
                                1.0f);
    float voltage_limit = c->nominal_limit_a * voltage_factor;

    float temp_factor = lerp(temp_c,
                             CURRENT_LIM_TEMP_NOMINAL,
                             CURRENT_LIM_TEMP_MAX,
                             1.0f,
                             0.30f);
    float temp_limit = c->nominal_limit_a * temp_factor;

    float dynamic_limit = softstart_limit;
    if (voltage_limit < dynamic_limit) dynamic_limit = voltage_limit;
    if (temp_limit    < dynamic_limit) dynamic_limit = temp_limit;

    c->current_limit_a = clamp(dynamic_limit,
                                c->min_limit_a,
                                c->peak_limit_a);
}

float current_limiter_apply(current_limiter_t *c, float pid_output)
{
    /* Eğer Limp Mode (override) aktifse, limit min_limit_a olur */
    float active_limit = c->is_override_active ? c->min_limit_a : c->current_limit_a;

    if (c->nominal_limit_a <= 0.0f)
        return 0.0f;

    /* Yüzdesel hesaplamayı aktif limite göre yap */
    float max_output = (active_limit / c->nominal_limit_a) * 100.0f;
    max_output = clamp(max_output, 0.0f, 100.0f);

    float limited_output;
    if (pid_output > max_output)
    {
        limited_output = max_output;
        c->limit_hit_count++;
    }
    else if (pid_output < -max_output)
    {
        limited_output = -max_output;
        c->limit_hit_count++;
    }
    else
    {
        limited_output = pid_output;
    }

    return limited_output;
}

float current_limiter_get_limit(const current_limiter_t *c)
{
    return c->is_override_active ? c->min_limit_a : c->current_limit_a;
}

uint32_t current_limiter_get_hit_count(const current_limiter_t *c)
{
    return c->limit_hit_count;
}
