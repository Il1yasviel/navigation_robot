#include "bmi088/bmi088_hal.h"

#include <stdbool.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* SPI2 总线通过 SPI_DMA_CH_AUTO 挂到 GDMA，所有 SPI 事务由 DMA 搬运。
 * 下面的静态 DMA_ATTR 缓冲区位于内部 SRAM 且 4 字节对齐，满足 IDF SPI
 * 主机驱动对 DMA 缓冲区的要求（spi_master.c setup_priv_desc），驱动因此
 * 不再为每笔事务临时分配 DMA 缓冲区并 memcpy——数据全程零拷贝。
 * 两个传感器共用一条总线，缓冲区由 s_buffer_mutex 串行化保护。 */
static DMA_ATTR uint8_t s_tx_buf[BMI08_MAX_LEN + 4];
static DMA_ATTR uint8_t s_rx_buf[BMI08_MAX_LEN + 4];
static SemaphoreHandle_t s_buffer_mutex;

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
    if (s_buffer_mutex == NULL) {
        s_buffer_mutex = xSemaphoreCreateMutex();
        if (s_buffer_mutex == NULL) return ESP_ERR_NO_MEM;
    }
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
    if (device == NULL || s_buffer_mutex == NULL) return BMI08_E_NULL_PTR;
    /* 传输长度补齐到 4 字节边界以满足 DMA 对齐要求。多出的时钟周期让
     * BMI088 自增输出后续寄存器内容，多收的字节直接丢弃，读取安全。 */
    const uint32_t padded_len = (len + 1u + 3u) & ~3u;
    xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
    memset(s_tx_buf, 0, padded_len);
    s_tx_buf[0] = reg_addr;
    spi_transaction_t transaction = {
        .length = padded_len * 8u,
        .tx_buffer = s_tx_buf,
        .rx_buffer = s_rx_buf,
    };
    const esp_err_t status = spi_device_transmit(device, &transaction);
    if (status == ESP_OK) memcpy(reg_data, &s_rx_buf[1], len);
    xSemaphoreGive(s_buffer_mutex);
    return status == ESP_OK ? BMI08_INTF_RET_SUCCESS : BMI08_E_COM_FAIL;
}

BMI08_INTF_RET_TYPE bmi088_hal_spi_write(uint8_t reg_addr,
                                         const uint8_t *reg_data,
                                         uint32_t len, void *intf_ptr)
{
    if (reg_data == NULL || intf_ptr == NULL) return BMI08_E_NULL_PTR;
    if (len > BMI08_MAX_LEN) return BMI08_E_RD_WR_LENGTH_INVALID;
    spi_device_handle_t device = device_for_sensor(*(uint8_t *)intf_ptr);
    if (device == NULL || s_buffer_mutex == NULL) return BMI08_E_NULL_PTR;
    /* 写不能补齐长度：多出的字节会真实写入后续寄存器。TX 缓冲区已满足
     * DMA 能力要求，长度不补齐。 */
    xSemaphoreTake(s_buffer_mutex, portMAX_DELAY);
    memset(s_tx_buf, 0, len + 1u);
    s_tx_buf[0] = reg_addr;
    memcpy(&s_tx_buf[1], reg_data, len);
    spi_transaction_t transaction = {
        .length = (len + 1u) * 8u,
        .tx_buffer = s_tx_buf,
    };
    const esp_err_t status = spi_device_transmit(device, &transaction);
    xSemaphoreGive(s_buffer_mutex);
    return status == ESP_OK ? BMI08_INTF_RET_SUCCESS : BMI08_E_COM_FAIL;
}

void bmi088_hal_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    esp_rom_delay_us(period);
}
