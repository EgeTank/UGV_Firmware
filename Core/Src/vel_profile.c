/* ============================================================
   vel_profile.c — Hız Profili Planlayıcı Implementasyonu

   MİMARİ NOT:
   Mevcut hiçbir dosyaya dokunulmaz.
   main.c PID döngüsünde sadece tek satır değişiklik:

     ESKİ: float target = motor_get_target_speed();
     YENİ: float target = vel_profile_step(&vel_prof,
                              motor_get_target_speed(), dt);

   vel_prof değişkeni main.c içinde tanımlanır ve
   main() içinde vel_profile_init() ile başlatılır.
   ============================================================ */

#include "vel_profile.h"

/* ============================================================
   YARDIMCI: float clamp
   ============================================================ */
static float clamp(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

/* ============================================================
   YARDIMCI: float abs
   ============================================================ */
static float fabsf_local(float v)
{
    return (v >= 0.0f) ? v : -v;
}

/* ============================================================
   vel_profile_init()
   ============================================================ */
void vel_profile_init(vel_profile_t *p,
                      float max_accel,
                      float max_decel,
                      float max_jerk,
                      vel_profile_mode_t mode)
{
    p->max_accel_mm_s2 = max_accel;
    p->max_decel_mm_s2 = max_decel;
    p->max_jerk_mm_s3  = max_jerk;
    p->mode            = mode;
    p->current_vel     = 0.0f;
    p->current_accel   = 0.0f;
}

/* ============================================================
   vel_profile_reset()
   Acil durum veya STATE_STANDBY → STATE_DRIVING geçişinde çağrılır.
   ============================================================ */
void vel_profile_reset(vel_profile_t *p)
{
    p->current_vel   = 0.0f;
    p->current_accel = 0.0f;
}

/* ============================================================
   vel_profile_is_settled()
   ============================================================ */
uint8_t vel_profile_is_settled(const vel_profile_t *p, float target_vel)
{
    /* 5 mm/s tolerans — titreşimi önler */
    return (fabsf_local(p->current_vel - target_vel) < 5.0f) ? 1u : 0u;
}

/* ============================================================
   TRAPEZOID PROFIL — iç fonksiyon

   Mantık:
     Hedefe yakınsa yavaşla (decel bölgesi),
     uzaktaysa ivmelen (accel bölgesi).
     Her adımda current_vel en fazla (accel * dt) kadar değişir.

   Duruş güvenliği:
     Hedef = 0 gelirse max_decel ile fren yapar.
     Negatif hız (geri) tam desteklenir.
   ============================================================ */
static float trapezoid_step(vel_profile_t *p, float target_vel, float dt)
{
    float error = target_vel - p->current_vel;

    /* Hangi ivme limitini kullanacağız?
       Hedefe doğru gidiyorsak accel,
       hedeften uzaklaşıyorsak (veya fren yapıyorsak) decel. */
    float accel_limit;
    if (fabsf_local(error) < 1.0f)
    {
        /* Hedete çok yakın — ani hareket yapma */
        p->current_vel = target_vel;
        return p->current_vel;
    }

    /* Fren mesafesi hesabı:
       v² = 2 * a * d  →  d = v² / (2*a)
       Şu anki hızla durabilmek için kalan mesafe yeterli mi? */
    float brake_dist = (p->current_vel * p->current_vel) /
                       (2.0f * p->max_decel_mm_s2 + 0.001f);

    float dist_to_target = fabsf_local(error);

    if (dist_to_target <= brake_dist)
    {
        /* Yavaşlama bölgesi — decel uygula */
        accel_limit = p->max_decel_mm_s2;
    }
    else
    {
        /* Hızlanma bölgesi — accel uygula */
        accel_limit = p->max_accel_mm_s2;
    }

    /* Bu adımda değişebilecek maksimum hız miktarı */
    float delta_max = accel_limit * dt;

    /* Hatayı delta_max ile sınırla */
    float delta = clamp(error, -delta_max, delta_max);

    p->current_vel += delta;

    return p->current_vel;
}

/* ============================================================
   S-CURVE PROFİL — iç fonksiyon

   Mantık:
     İvmenin kendisi de kademeli değişir (jerk kontrolü).
     current_accel, hedef ivmeye max_jerk * dt kadar yaklaşır.
     current_vel, current_accel * dt kadar değişir.

   Bu sayede:
     - Kalkışta ani tork yok → mekanik stres azalır
     - Duruşta ani fren yok → yolcu konforu / yük stabilitesi
     - %45 eğimde yumuşak kalkış kritik önem taşır

   Durum değişkenleri:
     current_vel   : anlık yumuşatılmış hız
     current_accel : anlık yumuşatılmış ivme
   ============================================================ */
static float scurve_step(vel_profile_t *p, float target_vel, float dt)
{
    float vel_error = target_vel - p->current_vel;

    /* Hedefe çok yakınsa sabitle */
    if (fabsf_local(vel_error) < 1.0f)
    {
        p->current_vel   = target_vel;
        p->current_accel = 0.0f;
        return p->current_vel;
    }

    /* Hedefe doğru hareket etmek için gereken ivme yönü */
    float desired_accel;

    /* Fren mesafesi: şu anki ivme de dahil */
    float brake_dist = (p->current_vel * p->current_vel) /
                       (2.0f * p->max_decel_mm_s2 + 0.001f);

    float dist_to_target = fabsf_local(vel_error);

    if (dist_to_target <= brake_dist)
    {
        /* Yavaşlama: negatif ivme (hedefe doğru) */
        desired_accel = (vel_error > 0.0f)
                        ? -p->max_decel_mm_s2
                        :  p->max_decel_mm_s2;
    }
    else
    {
        /* Hızlanma: hedef yönüne pozitif ivme */
        desired_accel = (vel_error > 0.0f)
                        ?  p->max_accel_mm_s2
                        : -p->max_accel_mm_s2;
    }

    /* İvmeyi jerk ile sınırla — S-curve'ün özü budur:
       current_accel, desired_accel'e max_jerk*dt kadar yaklaşır */
    float accel_error = desired_accel - p->current_accel;
    float jerk_limit  = p->max_jerk_mm_s3 * dt;
    float accel_delta = clamp(accel_error, -jerk_limit, jerk_limit);

    p->current_accel += accel_delta;

    /* İvmeyi fiziksel limitlerle sınırla */
    p->current_accel = clamp(p->current_accel,
                             -p->max_decel_mm_s2,
                              p->max_accel_mm_s2);

    /* Hızı ivmeyle güncelle */
    p->current_vel += p->current_accel * dt;

    return p->current_vel;
}

/* ============================================================
   vel_profile_step() — ANA FONKSİYON
   PID döngüsünden her 10ms'de bir çağrılır.
   ============================================================ */
float vel_profile_step(vel_profile_t *p, float target_vel, float dt)
{
    if (dt <= 0.0f || dt > 1.0f)
        return p->current_vel;   /* Geçersiz dt — mevcut hızı koru */

    switch (p->mode)
    {
        case VEL_PROFILE_TRAPEZOID:
            return trapezoid_step(p, target_vel, dt);

        case VEL_PROFILE_SCURVE:
            return scurve_step(p, target_vel, dt);

        default:
            return trapezoid_step(p, target_vel, dt);

    }
}

