#include "bmi088/bmi088_hal.h"

#include <stdbool.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_rom_sys.h"

static spi_device_handle_t s_accel_dev;
static spi_device_handle_t s_gyro_dev;
static bool s_initialized;

static spi_device_handle_t device_for_sensor(uint8_t sensor)
{
    return sensor == BMI088_HAL_SENSOR_ACC ? s_accel_dev : s_gyro_dev;
}

esp_err_t bmi088_hal_bus_init(void)
{
    if (s_initialized) return ESP_OK;
    const spi_bus_config_t bus = {
        .mosi_io_num = BMI088_HAL_PIN_MOSI,
        .miso_io_num = BMI088_HAL_PIN_MISO,
        .sclk_io_num = BMI088_HAL_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BMI08_MAX_LEN + 8,
    };
    esp_err_t status = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (status != ESP_OK) return status;
    const spi_device_interface_config_t common = {
        .mode = 0,
        .clock_speed_hz = 10000000,
        .queue_size = 2,
        .spics_io_num = -1,
    };
    spi_device_interface_config_t accel = common;
    accel.spics_io_num = BMI088_HAL_PIN_CS_ACC;
    status = spi_bus_add_device(SPI2_HOST, &accel, &s_accel_dev);
    if (status != ESP_OK) return status;
    spi_device_interface_config_t gyro = common;
    gyro.spics_io_num = BMI088_HAL_PIN_CS_GYRO;
    status = spi_bus_add_device(SPI2_HOST, &gyro, &s_gyro_dev);
    if (status == ESP_OK) s_initialized = true;
    return status;
}

BMI08_INTF_RET_TYPE bmi088_hal_spi_read(uint8_t reg_addr, uint8_t *reg_data,
                                        uint32_t len, void *intf_ptr)
{
    if (reg_data == NULL || intf_ptr == NULL) return BMI08_E_NULL_PTR;
    if (len == 0u || len > BMI08_MAX_LEN) return BMI08_E_RD_WR_LENGTH_INVALID;
    spi_device_handle_t device = device_for_sensor(*(uint8_t *)intf_ptr);
    if (device == NULL) return BMI08_E_NULL_PTR;
    uint8_t tx[BMI08_MAX_LEN + 1] = {0};
    uint8_t rx[BMI08_MAX_LEN + 1] = {0};
    tx[0] = reg_addr;
    spi_transaction_t transaction = {
        .length = (len + 1u) * 8u,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    if (spi_device_transmit(device, &transaction) != ESP_OK) {
        return BMI08_E_COM_FAIL;
    }
    memcpy(reg_data, &rx[1], len);
    return BMI08_INTF_RET_SUCCESS;
}

BMI08_INTF_RET_TYPE bmi088_hal_spi_write(uint8_t reg_addr,
                                         const uint8_t *reg_data,
                                         uint32_t len, void *intf_ptr)
{
    if (reg_data == NULL || intf_ptr == NULL) return BMI08_E_NULL_PTR;
    if (len > BMI08_MAX_LEN) return BMI08_E_RD_WR_LENGTH_INVALID;
    spi_device_handle_t device = device_for_sensor(*(uint8_t *)intf_ptr);
    if (device == NULL) return BMI08_E_NULL_PTR;
    uint8_t tx[BMI08_MAX_LEN + 1] = {0};
    tx[0] = reg_addr;
    memcpy(&tx[1], reg_data, len);
    spi_transaction_t transaction = {
        .length = (len + 1u) * 8u,
        .tx_buffer = tx,
    };
    return spi_device_transmit(device, &transaction) == ESP_OK
               ? BMI08_INTF_RET_SUCCESS : BMI08_E_COM_FAIL;
}

void bmi088_hal_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}
