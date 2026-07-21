/*
 * pid_tuner.c — PID İnce Ayar Mekanizması İmplementasyonu
 *
 * Bağımlılıklar: pid_tuner.h, pid.h
 */

#ifdef DEBUG_SIMULATION

#include "pid_tuner.h"

/* ============================================================
   YARDIMCILAR
   ============================================================ */
static float _pt_fabs(float v) { return (v >= 0.0f) ? v : -v; }

/* ============================================================
   pid_tuner_init()
   ============================================================ */
void pid_tuner_init(pid_tuner_t *t, float kp, float ki, float kd)
{
    t->kp = kp;
    t->ki = ki;
    t->kd = kd;

    t->setpoint_mm_s     = 0.0f;
    t->step_start_ms     = 0u;
    t->step_active       = 0u;
    t->rise_10_mm_s      = 0.0f;
    t->rise_90_mm_s      = 0.0f;
    t->rise_start_ms     = 0u;
    t->rise_done         = 0u;
    t->settle_enter_ms   = 0u;
    t->settle_done       = 0u;
    t->max_measured_mm_s = 0.0f;
    t->sse_idx           = 0u;
    t->rise_time_ms      = 0.0f;
    t->settle_time_ms    = 0.0f;
    t->overshoot_pct     = 0.0f;
    t->sse_mm_s          = 0.0f;
    t->metrics_valid     = 0u;

    for (uint8_t i = 0u; i < PID_TUNER_SSE_WINDOW_LEN; i++)
        t->sse_window[i] = 0.0f;
}

/* ============================================================
   pid_tuner_apply()
   ============================================================ */
void pid_tuner_apply(pid_tuner_t *t, PID_Controller_t *pid)
{
    pid->Kp = t->kp;
    pid->Ki = t->ki;
    pid->Kd = t->kd;
    PID_Reset(pid);
}

/* ============================================================
   pid_tuner_start_step()
   ============================================================ */
void pid_tuner_start_step(pid_tuner_t *t,
                          float setpoint_mm_s,
                          uint32_t now_ms)
{
    float abs_sp = _pt_fabs(setpoint_mm_s);

    t->setpoint_mm_s     = setpoint_mm_s;
    t->step_start_ms     = now_ms;
    t->step_active       = 1u;
    t->metrics_valid     = 0u;
    t->rise_done         = 0u;
    t->settle_done       = 0u;
    t->rise_start_ms     = 0u;
    t->settle_enter_ms   = 0u;
    t->max_measured_mm_s = 0.0f;
    t->sse_idx           = 0u;

    /* %10 ve %90 eşikleri */
    t->rise_10_mm_s = abs_sp * 0.10f;
    t->rise_90_mm_s = abs_sp * 0.90f;

    for (uint8_t i = 0u; i < PID_TUNER_SSE_WINDOW_LEN; i++)
        t->sse_window[i] = 0.0f;
}

/* ============================================================
   pid_tuner_step()

   Her 10ms'de ControlTask içinden çağrılır.
   Üç metriği paralel izler: rise, settle, SSE.
   ============================================================ */
void pid_tuner_step(pid_tuner_t *t,
                    float measured_mm_s,
                    uint32_t now_ms)
{
    if (!t->step_active) return;

    float abs_meas = _pt_fabs(measured_mm_s);
    float abs_sp   = _pt_fabs(t->setpoint_mm_s);

    /* ── Aşım takibi (sürekli) ── */
    if (abs_meas > t->max_measured_mm_s)
        t->max_measured_mm_s = abs_meas;

    /* ── Yükselme süresi ──
       %10'a ilk ulaşıldığında rise_start_ms kaydet.
       %90'a ilk ulaşıldığında rise_time_ms hesapla. */
    if (!t->rise_done)
    {
        if ((t->rise_start_ms == 0u) && (abs_meas >= t->rise_10_mm_s))
            t->rise_start_ms = now_ms;

        if ((t->rise_start_ms != 0u) && (abs_meas >= t->rise_90_mm_s))
        {
            t->rise_time_ms = (float)(now_ms - t->rise_start_ms);
            t->rise_done    = 1u;
        }
    }

    /* ── Yerleşme süresi ──
       Hata bant içine girince zamanlayıcı başlar.
       Bant dışına çıkılırsa zamanlayıcı sıfırlanır.
       PID_TUNER_SETTLE_MIN_MS boyunca aralıksız bant içinde kalınca yerleşti. */
    if (!t->settle_done && (abs_sp > 0.0f))
    {
        float band = abs_sp * (PID_TUNER_SETTLE_BAND_PCT / 100.0f);
        float err  = _pt_fabs(measured_mm_s - t->setpoint_mm_s);

        if (err <= band)
        {
            if (t->settle_enter_ms == 0u)
                t->settle_enter_ms = now_ms;

            if ((now_ms - t->settle_enter_ms) >= PID_TUNER_SETTLE_MIN_MS)
            {
                /* settle_time: step başından bant girişine kadar */
                t->settle_time_ms = (float)(t->settle_enter_ms - t->step_start_ms);
                t->settle_done    = 1u;
            }
        }
        else
        {
            /* Bant dışı — süreyi sıfırla */
            t->settle_enter_ms = 0u;
        }
    }

    /* ── SSE kayan pencere (sürekli güncellenir) ── */
    t->sse_window[t->sse_idx] = _pt_fabs(measured_mm_s - t->setpoint_mm_s);
    t->sse_idx = (uint8_t)((t->sse_idx + 1u) % PID_TUNER_SSE_WINDOW_LEN);

    float sse_sum = 0.0f;
    for (uint8_t i = 0u; i < PID_TUNER_SSE_WINDOW_LEN; i++)
        sse_sum += t->sse_window[i];
    t->sse_mm_s = sse_sum / (float)PID_TUNER_SSE_WINDOW_LEN;

    /* ── Metrikler tamamlandı mı? ──
       Rise VE settle ölçümü bitmişse hesapla. */
    if (t->rise_done && t->settle_done && !t->metrics_valid)
    {
        if (abs_sp > 0.0f)
        {
            t->overshoot_pct =
                ((t->max_measured_mm_s - abs_sp) / abs_sp) * 100.0f;
            /* Negatif aşım (undershoot) sıfır kabul edilir */
            if (t->overshoot_pct < 0.0f)
                t->overshoot_pct = 0.0f;
        }
        t->metrics_valid = 1u;
    }
}

/* ============================================================
   Accessor fonksiyonları
   ============================================================ */
float   pid_tuner_get_rise_time_ms(const pid_tuner_t *t)   { return t->rise_time_ms;   }
float   pid_tuner_get_settle_time_ms(const pid_tuner_t *t) { return t->settle_time_ms; }
float   pid_tuner_get_overshoot(const pid_tuner_t *t)      { return t->overshoot_pct;  }
float   pid_tuner_get_sse(const pid_tuner_t *t)            { return t->sse_mm_s;       }
uint8_t pid_tuner_metrics_valid(const pid_tuner_t *t)      { return t->metrics_valid;  }

void pid_tuner_reset_metrics(pid_tuner_t *t)
{
    /* Katsayıları koru, metrikleri sıfırla */
    float kp = t->kp, ki = t->ki, kd = t->kd;
    pid_tuner_init(t, kp, ki, kd);
}

#endif /* DEBUG_SIMULATION */
