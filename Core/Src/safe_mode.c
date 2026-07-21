#include "safe_mode.h"
#include "system_config.h"

uint8_t safe_mode_check(uint8_t current_errors, uint8_t current_details, vel_profile_t *vp, current_limiter_t *cl)
{
    /* Kritik hata varsa (hard latch veya E-STOP) acil durdur */
    if (ERR_IS_CRITICAL(current_errors)) {
        current_limiter_set_override(cl, 0);
        return 0;   // Emergency
    }

    /* Detay hataları kontrol et: düşük voltaj veya yüksek sıcaklık */
    if (current_details & (ERR_DETAIL_LOW_VOLTAGE | ERR_DETAIL_HIGH_TEMP)) {
        /* Limp Mode: ivmeyi düşür, akımı min'e çek */
        vp->max_accel_mm_s2 = VEL_PROFILE_MAX_ACCEL_MM_S2 * 0.1f;
        vp->max_decel_mm_s2 = VEL_PROFILE_MAX_DECEL_MM_S2 * 0.5f;
        current_limiter_set_override(cl, 1);
        return 1;   // Limp mode
    }

    /* Her şey normal: ivmeleri normale döndür, override'ı kapat */
    vp->max_accel_mm_s2 = VEL_PROFILE_MAX_ACCEL_MM_S2;
    vp->max_decel_mm_s2 = VEL_PROFILE_MAX_DECEL_MM_S2;
    current_limiter_set_override(cl, 0);
    return 2;   // OK
}
