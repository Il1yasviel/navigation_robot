#ifndef ROBOT_JOYSTICK_MAPPING_H
#define ROBOT_JOYSTICK_MAPPING_H

#include <stdint.h>

int16_t joystick_single_wheel_rpm(int16_t y_permille, uint16_t max_rpm);
void joystick_differential_rpm(int16_t x_permille,
                               int16_t y_permille,
                               uint16_t max_rpm,
                               int16_t *left_rpm,
                               int16_t *right_rpm);

#endif
