#ifndef ESP32_DRIVERS_SYSTEM_TIME_H
#define ESP32_DRIVERS_SYSTEM_TIME_H

#include <stdint.h>

uint32_t esp32_time_millis(void);
void esp32_delay_ms(uint32_t milliseconds);

#endif
