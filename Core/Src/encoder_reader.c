#include "encoder_reader.h"
#include "unit_conversion.h"

#define ENCODER_RESOLUTION 4096.0f
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_M * 3.14159265f * 1000.0f)

// ===============================
// Hızı mm/s cinsine çevirme
// ===============================
float encoder_calculate_speed_mm_s(int16_t delta_count, float delta_time_s) {
    if (delta_time_s < 0.001f) return 0.0f;

    // delta_count artık ODrive'dan gelen hesaplanmış bir değer olduğu için
    // doğrudan çözünürlük ile oranlanıp mm/s hızına dönüştürülür.
    float total_revolutions = (float)delta_count / ENCODER_RESOLUTION;
    float speed_mm_s = (total_revolutions * WHEEL_CIRCUMFERENCE_MM) / delta_time_s;

    return speed_mm_s;
}
