#ifndef ROBOT_ESP32_WIFI_TCP_H
#define ROBOT_ESP32_WIFI_TCP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*esp32_wifi_tcp_receive_fn)(void *context,
                                          const uint8_t *data,
                                          size_t length);
typedef void (*esp32_wifi_tcp_disconnect_fn)(void *context);

esp_err_t esp32_wifi_tcp_start(esp32_wifi_tcp_receive_fn receive,
                               esp32_wifi_tcp_disconnect_fn disconnect,
                               void *context);
esp_err_t esp32_wifi_tcp_write(const uint8_t *data, size_t length,
                               uint32_t timeout_ms);
bool esp32_wifi_tcp_connected(void);
bool esp32_wifi_tcp_has_ip(void);

#ifdef __cplusplus
}
#endif

#endif
