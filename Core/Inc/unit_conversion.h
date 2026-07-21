#ifndef UNIT_CONVERSION_H
#define UNIT_CONVERSION_H

#include "system_config.h"   /* WHEEL_DIAMETER_M, GEAR_REDUCTION_RATIO buradan gelir */

/*
  Birim dönüşüm yardımcıları
   Teker çapı ve redüksiyon oranı system_config.h'dan gelir.
   Donanım gelince system_config.h'dan tek yerden güncellenir.
   Bu modül saf matematik sağlar; donanıma/HAL'e bağımlı değildir.
 */

float wheel_rpm_to_mps(float wheel_rpm);
float mps_to_wheel_rpm(float speed_mps);
float motor_rpm_to_mps(float motor_rpm);
float mps_to_motor_rpm(float speed_mps);
float mps_to_mmps(float speed_mps);
float mmps_to_mps(float speed_mmps);

#endif
