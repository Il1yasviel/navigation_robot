idf_component_register(
    SRCS
        "src/motor_service.c"
        "src/host_link_service.c"
        "src/safety_service.c"
    INCLUDE_DIRS "include"
    REQUIRES common config esp32_drivers protocols algorithms freertos esp_timer
)
