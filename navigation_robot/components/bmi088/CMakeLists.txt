idf_component_register(
    SRCS
        "src/bmi088_hal.c"
        "src/bmi088_service.c"
        "vendor/bmi08a.c"
        "vendor/bmi08g.c"
        "vendor/bmi08xa.c"
    INCLUDE_DIRS "include" "vendor"
    REQUIRES config esp_driver_spi esp_timer freertos
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-function)
