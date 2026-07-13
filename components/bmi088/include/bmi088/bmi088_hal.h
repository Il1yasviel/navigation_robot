#ifndef ROBOT_BMI088_HAL_H
#define ROBOT_BMI088_HAL_H

#include <stdint.h>

#include "bmi08_defs.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMI088_HAL_PIN_SCK     12
#define BMI088_HAL_PIN_MOSI    11
#define BMI088_HAL_PIN_MISO    13
#define BMI088_HAL_PIN_CS_ACC  47
#define BMI088_HAL_PIN_CS_GYRO 21

typedef enum {
    BMI088_HAL_SENSOR_ACC = 0,
    BMI088_HAL_SENSOR_GYRO = 1,
} bmi088_hal_sensor_t;

esp_err_t bmi088_hal_bus_init(void);
BMI08_INTF_RET_TYPE bmi088_hal_spi_read(uint8_t reg_addr, uint8_t *reg_data,
                                        uint32_t len, void *intf_ptr);
BMI08_INTF_RET_TYPE bmi088_hal_spi_write(uint8_t reg_addr,
                                         const uint8_t *reg_data,
                                         uint32_t len, void *intf_ptr);
void bmi088_hal_delay_us(uint32_t period, void *intf_ptr);

#ifdef __cplusplus
}
#endif

#endif
