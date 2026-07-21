#include "unit_conversion.h"

/* M_PI her toolchain'de gelmeyebiliyor */
#define UNIT_PI (3.14159265358979323846f)

static float wheel_circumference_m(void)
{
    return UNIT_PI * WHEEL_DIAMETER_M;
}

float wheel_rpm_to_mps(float wheel_rpm)
{
    /* v = (wheel_rpm / 60) * circumference */
    return (wheel_rpm / 60.0f) * wheel_circumference_m();
}

float mps_to_wheel_rpm(float speed_mps)
{
    /* wheel_rpm = (v / circumference) * 60 */
    float c = wheel_circumference_m();
    if (c <= 0.0f)
        return 0.0f;
    return (speed_mps / c) * 60.0f;
}

float motor_rpm_to_mps(float motor_rpm)
{
    /* wheel_rpm = motor_rpm / gear_ratio */
    if (GEAR_REDUCTION_RATIO <= 0.0f)
        return 0.0f;
    float wheel_rpm = motor_rpm / GEAR_REDUCTION_RATIO;
    return wheel_rpm_to_mps(wheel_rpm);
}

float mps_to_motor_rpm(float speed_mps)
{
    /* motor_rpm = wheel_rpm * gear_ratio */
    return mps_to_wheel_rpm(speed_mps) * GEAR_REDUCTION_RATIO;
}

float mps_to_mmps(float speed_mps)
{
    return speed_mps * 1000.0f;
}

float mmps_to_mps(float speed_mmps)
{
    return speed_mmps / 1000.0f;
}
