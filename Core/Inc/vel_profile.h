#ifndef VEL_PROFILE_H
#define VEL_PROFILE_H

#include <stdint.h>

typedef enum {
    VEL_PROFILE_TRAPEZOID = 0,   /* Sabit ivmeli trapez profil  */
    VEL_PROFILE_SCURVE    = 1    /* Jerk kontrollü S-eğrisi     */
} vel_profile_mode_t;

/* -------------------------------------------------------
   Planlayıcı yapısı
   ------------------------------------------------------- */
typedef struct {
    /* Ayarlar */
    float max_accel_mm_s2;    /* Maksimum ivme    (mm/s²) */
    float max_decel_mm_s2;    /* Maksimum yavaşlama (mm/s²) */
    float max_jerk_mm_s3;     /* Maksimum jerk    (mm/s³) — S-curve için */
    vel_profile_mode_t mode;  /* Trapezoid veya S-curve */

    /* İç durum */
    float current_vel;        /* Şu anki yumuşatılmış hız (mm/s) */
    float current_accel;      /* Şu anki ivme — S-curve için      */
} vel_profile_t;

/**
 * @brief  Planlayıcıyı başlat.
 * @param  p             Planlayıcı pointer'ı
 * @param  max_accel     Maksimum ivme (mm/s²)    — örn: 500.0f
 * @param  max_decel     Maksimum yavaşlama (mm/s²) — örn: 800.0f
 * @param  max_jerk      Maksimum jerk (mm/s³)    — örn: 2000.0f (S-curve)
 * @param  mode          VEL_PROFILE_TRAPEZOID veya VEL_PROFILE_SCURVE
 */
void vel_profile_init(vel_profile_t *p,
                      float max_accel,
                      float max_decel,
                      float max_jerk,
                      vel_profile_mode_t mode);

/**
 * @brief  Her PID döngüsünde çağrılır — yumuşatılmış hedef hızı döner.
 * @param  p          Planlayıcı pointer'ı
 * @param  target_vel Hedef hız (mm/s) — ROS'tan gelen ham değer
 * @param  dt         Geçen süre (saniye) — PID döngüsüyle aynı dt
 * @retval Yumuşatılmış hedef hız (mm/s) — doğrudan PID'e verilir
 */
float vel_profile_step(vel_profile_t *p, float target_vel, float dt);

/**
 * @brief  Planlayıcıyı sıfırla (acil durum veya yeniden başlatma).
 *         current_vel ve current_accel sıfırlanır.
 */
void vel_profile_reset(vel_profile_t *p);

/**
 * @brief  Hedefe ulaşıldı mı?
 * @retval 1: hedefte, 0: hâlâ rampa devam ediyor
 */
uint8_t vel_profile_is_settled(const vel_profile_t *p, float target_vel);

#endif /* VEL_PROFILE_H */
