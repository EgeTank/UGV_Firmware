#ifndef SAFE_MODE_H
#define SAFE_MODE_H

#include <stdint.h>
#include "vel_profile.h"
#include "current_limiter.h"
#include "error_codes.h" /* safe_mode_check içindeki ERR_ macrosunu kullanabilmek için gerekli */


uint8_t safe_mode_check(uint8_t current_errors, uint8_t current_details, vel_profile_t *vp, current_limiter_t *cl);

#endif /* SAFE_MODE_H */
