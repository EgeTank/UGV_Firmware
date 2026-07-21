#ifndef DRIVE_MODE_H
#define DRIVE_MODE_H

#include <stdint.h>
#include "vel_profile.h"
#include "current_limiter.h"

typedef enum {
    MODE_NORMAL    = 0,  /* Standart sürüş */
    MODE_PRECISION = 1,  /* Hedefleme/Taret modu - Düşük hız, yumuşak ivme */
    MODE_AGGRESSIVE= 2,  /* Düzlük modu - Yüksek hız, sert ivme */
    MODE_CLIMB     = 3   /* Tırmanma modu - Yüksek tork, düşük hız */
} drive_mode_t;

void drive_mode_update(uint8_t surus_modu, vel_profile_t *vp, current_limiter_t *cl);

#endif /* DRIVE_MODE_H */
