#include "esp32_drivers/host_uart.h"

#include "config/robot_config.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

static uart_port_t host_port(void)
{
    return (uart_port_t)CONFIG_ROBOT_HOST_UART_NUM;
}

esp_err_t esp32_host_uart_init(void)
{
    const uart_port_t port = host_port();
    const uart_config_t uart_config = {
        .baud_rate = ROBOT_HOST_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t status = uart_driver_install(port,
                                            ROBOT_HOST_RX_BUFFER_SIZE,
                                            ROBOT_HOST_TX_BUFFER_SIZE,
                                            0,
                                            NULL,
                                            0);
    if (status != ESP_OK) {
        return status;
    }

    status = uart_param_config(port, &uart_config);
    if (status == ESP_OK) {
        status = uart_set_pin(port,
                              CONFIG_ROBOT_HOST_UART_TX_GPIO,
                              CONFIG_ROBOT_HOST_UART_RX_GPIO,
                              UART_PIN_NO_CHANGE,
                              UART_PIN_NO_CHANGE);
    }
    if (status != ESP_OK) {
        (void)uart_driver_delete(port);
        return status;
    }

    return uart_flush_input(port);
}

esp_err_t esp32_host_uart_read(uint8_t *data,
                               size_t capacity,
                               uint32_t timeout_ms,
                               size_t *bytes_read)
{
    if (data == NULL || capacity == 0u || bytes_read == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const int received = uart_read_bytes(host_port(),
                                          data,
                                          capacity,
                                          pdMS_TO_TICKS(timeout_ms));
    if (received < 0) {
        *bytes_read = 0u;
        return ESP_FAIL;
    }

    *bytes_read = (size_t)received;
    return received == 0 ? ESP_ERR_TIMEOUT : ESP_OK;
}

esp_err_t esp32_host_uart_write(const uint8_t *data,
                                size_t length,
                                uint32_t timeout_ms)
{
    if (data == NULL || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    const uart_port_t port = host_port();
    const int written = uart_write_bytes(port, data, length);
    if (written != (int)length) {
        return ESP_FAIL;
    }
    return uart_wait_tx_done(port, pdMS_TO_TICKS(timeout_ms));
}
