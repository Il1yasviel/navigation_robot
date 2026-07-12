#ifndef ESP32_DRIVERS_RS485_UART_H
#define ESP32_DRIVERS_RS485_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

typedef struct {
    uart_port_t uart_port;
    bool initialized;
} esp32_rs485_t;

esp_err_t esp32_rs485_init(esp32_rs485_t *bus);
esp_err_t esp32_rs485_send(esp32_rs485_t *bus,
                           const uint8_t *data,
                           size_t length,
                           uint32_t timeout_ms);
esp_err_t esp32_rs485_receive_exact(esp32_rs485_t *bus,
                                    uint8_t *data,
                                    size_t length,
                                    uint32_t timeout_ms);
void esp32_rs485_flush_rx(esp32_rs485_t *bus);

#endif
