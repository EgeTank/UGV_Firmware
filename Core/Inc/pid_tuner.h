

#ifndef PID_TUNER_H
#define PID_TUNER_H

#ifdef DEBUG_SIMULATION

#include <stdint.h>
#include "pid.h"   /* PID_Controller_t, PID_Reset() */

/** Yerleşme bandı: setpoint'in ±%N'i */
#define PID_TUNER_SETTLE_BAND_PCT   5.0f

/** Yerleşme için bu kadar ms aralıksız bantta kalmalı */
#define PID_TUNER_SETTLE_MIN_MS     200U

/** SSE hesabı için kayan pencere uzunluğu (örnekler) */
#define PID_TUNER_SSE_WINDOW_LEN    50U

/* ============================================================
   VERİ YAPISI
   ============================================================ */
typedef struct {
    /* Katsayılar */
    float kp;
    float ki;
    float kd;

    /* Step tanımı */
    float    setpoint_mm_s;       /* Test setpoint'i (mm/s)             */
    uint32_t step_start_ms;       /* pid_tuner_start_step() anı         */
    uint8_t  step_active;         /* Test sürüyor mu?                   */

    /* Yükselme süresi yardımcıları */
    float    rise_10_mm_s;        /* %10 eşiği                          */
    float    rise_90_mm_s;        /* %90 eşiği                          */
    uint32_t rise_start_ms;       /* %10'a ilk ulaşılan tick            */
    uint8_t  rise_done;           /* %90'a ulaşıldı mı?                 */

    /* Yerleşme yardımcıları */
    uint32_t settle_enter_ms;     /* Banda ilk giriş tick'i             */
    uint8_t  settle_done;         /* 200ms boyunca bantta kaldı mı?     */

    /* Aşım yardımcısı */
    float    max_measured_mm_s;   /* Görülen maksimum hız (mutlak)      */

    /* SSE kayan pencere */
    float    sse_window[PID_TUNER_SSE_WINDOW_LEN];
    uint8_t  sse_idx;

    /* ── SONUÇLAR ── */
    float   rise_time_ms;         /* %10 → %90 süresi (ms)             */
    float   settle_time_ms;       /* Başlangıçtan yerleşmeye (ms)       */
    float   overshoot_pct;        /* Aşım yüzdesi (%)                   */
    float   sse_mm_s;             /* Kararlı-durum ort. mutlak hata     */
    uint8_t metrics_valid;        /* 1 → ölçüm tamamlandı              */
} pid_tuner_t;


/** @brief PID tuner'ı başlatır. */
void pid_tuner_init(pid_tuner_t *t, float kp, float ki, float kd);

/**
 * @brief  Katsayıları PID controller'a yazar ve reset atar.
 *         pid_tuner_start_step()'ten önce çağrılmalı.
 */
void pid_tuner_apply(pid_tuner_t *t, PID_Controller_t *pid);


void pid_tuner_start_step(pid_tuner_t *t,
                          float setpoint_mm_s,
                          uint32_t now_ms);

tuner_step(pid_tuner_t *t,
                    float measured_mm_s,
                    uint32_t now_ms);

/* ── Sonuç okuyucular ── */
float    pid_tuner_get_rise_time_ms(const pid_tuner_t *t);
float    pid_tuner_get_settle_time_ms(const pid_tuner_t *t);
float    pid_tuner_get_overshoot(const pid_tuner_t *t);
float    pid_tuner_get_sse(const pid_tuner_t *t);

/** @brief 1 → rise + settle ölçümleri tamamlandı. */
uint8_t  pid_tuner_metrics_valid(const pid_tuner_t *t);

/** @brief Metrikleri sıfırlar, katsayıları korur. */
void pid_tuner_reset_metrics(pid_tuner_t *t);

#endif /* DEBUG_SIMULATION */

#endif /* PID_TUNER_H */
