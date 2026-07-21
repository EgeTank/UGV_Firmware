/*
 * imu_filter.c — IMU Tamamlayıcı Filtre Implementasyonu
 *
 * Bağımlılıklar: imu_filter.h, <stdint.h>
 * math.h kullanılmaz — bare-metal uyumluluğu için Bhaskara sin yaklaşımı.
 */

#include "imu_filter.h"

/* ============================================================
   YARDIMCILAR (statik — dışarıya açılmaz)
   ============================================================ */
static float _clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* ============================================================
   imu_filter_init()
   ============================================================ */
void imu_filter_init(imu_filter_t *f, float alpha)
{
    f->angle_deg   = 0.0f;
    f->alpha       = _clamp(alpha, 0.0f, 1.0f);
    f->initialized = 0u;
}

/* ============================================================
   imu_filter_update()

   Tamamlayıcı filtre:
     Yüksek-geçiren (gyro) — kısa dönem, drift birikmez
     Düşük-geçiren  (accel) — uzun dönem, drift iptal

   İlk çağrıda gyro kullanılmaz, accel ile başlatılır.
   Bu sayede güç açılışında kaba gürültü filtreye işlenmez.
   ============================================================ */
float imu_filter_update(imu_filter_t *f,
                        float gyro_dps,
                        float accel_angle_deg,
                        float dt_s)
{
    if (dt_s <= 0.0f) return f->angle_deg;

    if (!f->initialized)
    {
        f->angle_deg   = accel_angle_deg;
        f->initialized = 1u;
        return f->angle_deg;
    }

    /* angle = α × (angle + ω × dt)  +  (1-α) × θ_accel */
    float gyro_pred  = f->angle_deg + gyro_dps * dt_s;
    f->angle_deg     = f->alpha * gyro_pred
                     + (1.0f - f->alpha) * accel_angle_deg;

    return f->angle_deg;
}

/* ============================================================
   imu_filter_get_angle()
   ============================================================ */
float imu_filter_get_angle(const imu_filter_t *f)
{
    return f->angle_deg;
}

/* ============================================================
   imu_filter_reset()
   ============================================================ */
void imu_filter_reset(imu_filter_t *f, float initial_angle_deg)
{
    f->angle_deg   = initial_angle_deg;
    f->initialized = 1u;
}


/* ============================================================
   DEBUG_SIMULATION — Fake IMU Veri Üreteci
   ============================================================ */
#ifdef DEBUG_SIMULATION

/* -------------------------------------------------------
   LCG Pseudo-random number generator
   MISRA C uyumlu — harici kütüphane gerektirmez.
   ------------------------------------------------------- */
static uint32_t s_rng = 0x12345678u;

static float _rand_f(void)
{
    s_rng ^= s_rng << 13u;
    s_rng ^= s_rng >> 17u;
    s_rng ^= s_rng << 5u;
    /* [-1.0, 1.0] aralığı */
    return (float)(int32_t)s_rng / 2147483647.0f;
}

/* -------------------------------------------------------
   Bhaskara I sin yaklaşımı
   Hata: < 0.17% — math.h gerektirmez
   Giriş: herhangi bir gerçek sayı (radyan)
   ------------------------------------------------------- */
static float _sin_approx(float x)
{
    const float PI     = 3.14159265f;
    const float TWO_PI = 6.28318530f;

    /* [0, 2π] aralığına taşı */
    while (x >  TWO_PI) x -= TWO_PI;
    while (x <  0.0f)   x += TWO_PI;

    float x2  = (x <= PI) ? x : (x - PI);
    float num = 4.0f * x2 * (PI - x2);
    float den = 5.0f * PI * PI - 4.0f * x2 * (PI - x2);
    float s   = num / den;
    return (x <= PI) ? s : -s;
}

static float _cos_approx(float x)
{
    return _sin_approx(x + 1.57079632f);   /* cos(x) = sin(x + π/2) */
}

/* -------------------------------------------------------
   imu_sim_init()
   ------------------------------------------------------- */
void imu_sim_init(imu_sim_state_t *sim, float noise_scale, uint8_t inject_drift)
{
    sim->sim_time_s   = 0.0f;
    sim->noise_scale  = (noise_scale >= 0.0f) ? noise_scale : 0.0f;
    sim->inject_drift = inject_drift;
}

/* -------------------------------------------------------
   imu_sim_step()

   Gerçek açı modeli: 45° × sin(2π × 0.1Hz × t)
     Araç bir rampaya yavaşça girip çıkıyor simülasyonu.

   Gyro  = d(açı)/dt + gürültü + sabit drift (inject_drift=1)
   Accel = gerçek açı + 3× gyro gürültüsü

   Filtre beklenen davranış:
     - Gyro ani değişimleri takip eder (rise time küçük).
     - Accel gyro driftini uzun vadede sıfırlar.
   ------------------------------------------------------- */
void imu_sim_step(imu_sim_state_t *sim, float dt_s,
                  float *out_gyro_dps,
                  float *out_accel_angle_deg)
{
    const float AMPLITUDE_DEG = 45.0f;
    const float FREQ_HZ       = 0.1f;
    const float TWO_PI        = 6.28318530f;

    float omega      = TWO_PI * FREQ_HZ;
    float true_angle = AMPLITUDE_DEG * _sin_approx(omega * sim->sim_time_s);
    float true_rate  = AMPLITUDE_DEG * omega * _cos_approx(omega * sim->sim_time_s);

    /* Gyro: türev + gürültü + opsiyonel drift */
    float drift       = sim->inject_drift ? 0.5f : 0.0f;
    *out_gyro_dps     = true_rate
                      + _rand_f() * sim->noise_scale
                      + drift;

    /* Accel: gerçek açı + 3× daha büyük gürültü */
    *out_accel_angle_deg = true_angle
                         + _rand_f() * (sim->noise_scale * 3.0f);

    sim->sim_time_s += dt_s;
}

#endif /* DEBUG_SIMULATION */


/*simülasyon aşamasında olduğu için gerçek  sensör entegrasyonu yok donanım gelince I2C okumasıyla doldurulacak.
 *DEBUG_SIMULATION aktif olduğu için imu_sim_step() çalışıyor ve değerleri dolduruyor.
 *0.0f değerleri üzerine yazılıyor.
 *
 *
 *
 *
 * */

