idf_component_register(
    SRCS
        "src/host_uart.c"
        "src/rs485_uart.c"
        "src/system_time.c"
        "src/wifi_tcp.c"
    INCLUDE_DIRS "include"
    REQUIRES config esp_driver_uart esp_timer esp_wifi esp_netif nvs_flash lwip mdns freertos
)
