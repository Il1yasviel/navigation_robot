#include "esp32_drivers/rs485_uart.h"

#include "config/robot_config.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

esp_err_t esp32_rs485_init(esp32_rs485_t *bus)
{
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const uart_port_t port = (uart_port_t)CONFIG_ROBOT_MOTOR_UART_NUM;
    const uart_config_t uart_config = {
        .baud_rate = ROBOT_MOTOR_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    bus->uart_port = port;
    bus->initialized = false;

    esp_err_t status = uart_driver_install(port, 256, 0, 0, NULL, 0);
    if (status != ESP_OK) {
        return status;
    }

    status = uart_param_config(port, &uart_config);
    if (status == ESP_OK) {
        status = uart_set_pin(port,
                              CONFIG_ROBOT_RS485_TX_GPIO,
                              CONFIG_ROBOT_RS485_RX_GPIO,
                              UART_PIN_NO_CHANGE,
                              UART_PIN_NO_CHANGE);
    }
    if (status == ESP_OK) {
        /* The isolated TTL-to-RS485 module switches TX/RX automatically. */
        status = uart_set_mode(port, UART_MODE_UART);
    }
    if (status != ESP_OK) {
        (void)uart_driver_delete(port);
        return status;
    }

    bus->initialized = true;
    return uart_flush_input(port);
}

void esp32_rs485_flush_rx(esp32_rs485_t *bus)
{
    if (bus != NULL && bus->initialized) {
        (void)uart_flush_input(bus->uart_port);
    }
}

esp_err_t esp32_rs485_send(esp32_rs485_t *bus,
                           const uint8_t *data,
                           size_t length,
                           uint32_t timeout_ms)
{
    if (bus == NULL || !bus->initialized || data == NULL || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    const uart_port_t port = bus->uart_port;
    esp_err_t status = uart_flush_input(port);
    if (status != ESP_OK) {
        return status;
    }
    const int written = uart_write_bytes(port, data, length);
    if (written != (int)length) {
        return ESP_FAIL;
    }
    return uart_wait_tx_done(port, pdMS_TO_TICKS(timeout_ms));
}

esp_err_t esp32_rs485_receive_exact(esp32_rs485_t *bus,
                                    uint8_t *data,
                                    size_t length,
                                    uint32_t timeout_ms)
{
    if (bus == NULL || !bus->initialized || data == NULL || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    const int64_t deadline_us = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    size_t received = 0;
    while (received < length) {
        const int64_t remaining_us = deadline_us - esp_timer_get_time();
        if (remaining_us <= 0) {
            return ESP_ERR_TIMEOUT;
        }
        TickType_t wait_ticks = pdMS_TO_TICKS((remaining_us + 999) / 1000);
        if (wait_ticks == 0) {
            wait_ticks = 1;
        }
        const int chunk = uart_read_bytes(bus->uart_port,
                                          &data[received],
                                          length - received,
                                          wait_ticks);
        if (chunk < 0) {
            return ESP_FAIL;
        }
        received += (size_t)chunk;
    }
    return ESP_OK;
}
