#ifndef SAFETY_H
#define SAFETY_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

// Safety durum tipleri
typedef enum {
    SAFETY_OK = 0,
    SAFETY_ESTOP_ACTIVE,
    SAFETY_BMS_FAULT
} safety_status_t;

// Safety modülü başlatma (GPIO vs. CubeMX tarafından zaten ayarlanıyor)
void Safety_Init(void);

// Safety kontrolü: E-Stop ve BMS durumunu kontrol eder
safety_status_t Safety_Check(void);

#endif // SAFETY_H
