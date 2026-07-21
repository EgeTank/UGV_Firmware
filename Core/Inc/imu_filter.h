#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdint.h>

typedef struct {
    float   angle_deg;       /* Filtre çıkışı — eğim açısı (derece)   */
    float   alpha;           /* Tamamlayıcı katsayı (0.0 – 1.0)       */
    uint8_t initialized;     /* İlk ölçüm alındı mı?                   */
} imu_filter_t;


void imu_filter_init(imu_filter_t *f, float alpha);


float imu_filter_update(imu_filter_t *f,
                        float gyro_dps,
                        float accel_angle_deg,
                        float dt_s);

/** @brief Son filtrelenmiş açıyı döner (derece). */
float imu_filter_get_angle(const imu_filter_t *f);

void imu_filter_reset(imu_filter_t *f, float initial_angle_deg);

#ifdef DEBUG_SIMULATION

typedef struct {
    float   sim_time_s;      /* Simülasyon zamanı (saniye)      */
    float   noise_scale;     /* Gürültü büyüklüğü (derece)      */
    uint8_t inject_drift;    /* 1 → 0.5 dps sabit gyro drift   */
} imu_sim_state_t;


void imu_sim_init(imu_sim_state_t *sim, float noise_scale, uint8_t inject_drift);


void imu_sim_step(imu_sim_state_t *sim, float dt_s,
                  float *out_gyro_dps, float *out_accel_angle_deg);

#endif /* DEBUG_SIMULATION */

#endif /* IMU_FILTER_H */
