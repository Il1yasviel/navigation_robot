#ifndef ESP32_DRIVERS_HOST_UART_H
#define ESP32_DRIVERS_HOST_UART_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t esp32_host_uart_init(void);
esp_err_t esp32_host_uart_read(uint8_t *data,
                               size_t capacity,
                               uint32_t timeout_ms,
                               size_t *bytes_read);
esp_err_t esp32_host_uart_write(const uint8_t *data,
                                size_t length,
                                uint32_t timeout_ms);

#endif
