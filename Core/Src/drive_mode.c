#include "drive_mode.h"
#include "system_config.h"

void drive_mode_update(uint8_t surus_modu, vel_profile_t *vp, current_limiter_t *cl)
{
    switch ((drive_mode_t)surus_modu) {
        case MODE_PRECISION:
            vp->max_accel_mm_s2 = VEL_PROFILE_MAX_ACCEL_MM_S2 * 0.3f;
            vp->max_decel_mm_s2 = VEL_PROFILE_MAX_DECEL_MM_S2 * 0.5f;
            vp->max_jerk_mm_s3  = VEL_PROFILE_MAX_JERK_MM_S3 * 0.2f;
            cl->peak_limit_a    = CURRENT_LIM_NOMINAL_A * 0.8f;
            break;

        case MODE_AGGRESSIVE:
            vp->max_accel_mm_s2 = VEL_PROFILE_MAX_ACCEL_MM_S2 * 1.5f;
            vp->max_decel_mm_s2 = VEL_PROFILE_MAX_DECEL_MM_S2 * 1.5f;
            vp->max_jerk_mm_s3  = VEL_PROFILE_MAX_JERK_MM_S3 * 2.0f;
            cl->peak_limit_a    = CURRENT_LIM_PEAK_A;
            break;

        case MODE_CLIMB:
            vp->max_accel_mm_s2 = VEL_PROFILE_MAX_ACCEL_MM_S2 * 0.5f;
            vp->max_decel_mm_s2 = VEL_PROFILE_MAX_DECEL_MM_S2 * 1.2f;
            vp->max_jerk_mm_s3  = VEL_PROFILE_MAX_JERK_MM_S3 * 0.5f;
            cl->peak_limit_a    = CURRENT_LIM_PEAK_A; /* Tork için yüksek akım izni */
            break;

        case MODE_NORMAL:
        default:
            vp->max_accel_mm_s2 = VEL_PROFILE_MAX_ACCEL_MM_S2;
            vp->max_decel_mm_s2 = VEL_PROFILE_MAX_DECEL_MM_S2;
            vp->max_jerk_mm_s3  = VEL_PROFILE_MAX_JERK_MM_S3;
            cl->peak_limit_a    = CURRENT_LIM_PEAK_A;
            break;
    }
}
