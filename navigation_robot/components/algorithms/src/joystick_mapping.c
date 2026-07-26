#include "algorithms/joystick_mapping.h"

#include <stddef.h>

#include "config/robot_config.h"

static int16_t clamp_permille(int16_t value)
{
    if (value > 1000) {
        return 1000;
    }
    if (value < -1000) {
        return -1000;
    }
    return value;
}

static int16_t clamp_rpm(int32_t value, uint16_t maximum)
{
    if (maximum > CONFIG_ROBOT_TEST_MAX_RPM) {
        maximum = CONFIG_ROBOT_TEST_MAX_RPM;
    }
    if (value > maximum) {
        return (int16_t)maximum;
    }
    if (value < -(int32_t)maximum) {
        return -(int16_t)maximum;
    }
    return (int16_t)value;
}

int16_t joystick_single_wheel_rpm(int16_t y_permille, uint16_t max_rpm)
{
    const int32_t scaled = (int32_t)clamp_permille(y_permille) * max_rpm / 1000;
    return clamp_rpm(scaled, max_rpm);
}

void joystick_differential_rpm(int16_t x_permille,
                               int16_t y_permille,
                               uint16_t max_rpm,
                               int16_t *left_rpm,
                               int16_t *right_rpm)
{
    const int32_t x = clamp_permille(x_permille);
    const int32_t y = clamp_permille(y_permille);

    if (left_rpm != NULL) {
        *left_rpm = clamp_rpm((y + x) * max_rpm / 1000, max_rpm);
    }
    if (right_rpm != NULL) {
        *right_rpm = clamp_rpm((y - x) * max_rpm / 1000, max_rpm);
    }
}
