#include "bmi088/bmi088_service.h"

#include <string.h>

#include "bmi08x.h"
#include "bmi088/bmi088_hal.h"
#include "config/robot_config.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define BMI088_RW_LENGTH 46u
#define BMI088_ACCEL_LSB_PER_G (32768.0f / 6.0f)
#define BMI088_GYRO_LSB_PER_DPS 65.536f
#define STANDARD_GRAVITY_MPS2 9.80665f
#define DEGREES_TO_RADIANS 0.01745329251994329577f

static struct bmi08_dev s_device;
static uint8_t s_accel_id = BMI088_HAL_SENSOR_ACC;
static uint8_t s_gyro_id = BMI088_HAL_SENSOR_GYRO;
static SemaphoreHandle_t s_mutex;
static bmi088_snapshot_t s_snapshot;

static esp_err_t initialize_device(void)
{
    esp_err_t status = bmi088_hal_bus_init();
    if (status != ESP_OK) return status;
    memset(&s_device, 0, sizeof(s_device));
    s_device.intf = BMI08_SPI_INTF;
    s_device.variant = BMI088_VARIANT;
    s_device.read = bmi088_hal_spi_read;
    s_device.write = bmi088_hal_spi_write;
    s_device.delay_us = bmi088_hal_delay_us;
    s_device.intf_ptr_accel = &s_accel_id;
    s_device.intf_ptr_gyro = &s_gyro_id;
    s_device.read_write_len = BMI088_RW_LENGTH;
    if (bmi08xa_init(&s_device) != BMI08_OK ||
        bmi08g_init(&s_device) != BMI08_OK) return ESP_FAIL;
    s_device.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    if (bmi08a_set_power_mode(&s_device) != BMI08_OK) return ESP_FAIL;
    s_device.accel_cfg.odr = BMI08_ACCEL_ODR_200_HZ;
    s_device.accel_cfg.bw = BMI08_ACCEL_BW_OSR2;
    s_device.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
    if (bmi08xa_set_meas_conf(&s_device) != BMI08_OK) return ESP_FAIL;
    s_device.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    if (bmi08g_set_power_mode(&s_device) != BMI08_OK) return ESP_FAIL;
    s_device.gyro_cfg.odr = BMI08_GYRO_BW_64_ODR_200_HZ;
    s_device.gyro_cfg.range = BMI08_GYRO_RANGE_500_DPS;
    return bmi08g_set_meas_conf(&s_device) == BMI08_OK ? ESP_OK : ESP_FAIL;
}

static esp_err_t read_raw(struct bmi08_sensor_data *accel,
                          struct bmi08_sensor_data *gyro)
{
    if (bmi08a_get_data(accel, &s_device) != BMI08_OK) return ESP_FAIL;
    return bmi08g_get_data(gyro, &s_device) == BMI08_OK ? ESP_OK : ESP_FAIL;
}

static void publish_sample(const struct bmi08_sensor_data *accel,
                           const struct bmi08_sensor_data *gyro,
                           const float bias[3])
{
    const int16_t accel_raw[3] = {accel->x, accel->y, accel->z};
    const int16_t gyro_raw[3] = {gyro->x, gyro->y, gyro->z};
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.timestamp_us = (uint64_t)esp_timer_get_time();
    for (size_t i = 0; i < 3; ++i) {
        s_snapshot.accel_mps2[i] =
            (float)accel_raw[i] * STANDARD_GRAVITY_MPS2 / BMI088_ACCEL_LSB_PER_G;
        s_snapshot.gyro_rads[i] =
            ((float)gyro_raw[i] - bias[i]) * DEGREES_TO_RADIANS /
            BMI088_GYRO_LSB_PER_DPS;
    }
    ++s_snapshot.sample_count;
    s_snapshot.online = true;
    s_snapshot.calibrated = true;
    s_snapshot.sample_valid = true;
    xSemaphoreGive(s_mutex);
}

static esp_err_t calibrate_gyro(float bias[3])
{
    int64_t sums[3] = {0};
    struct bmi08_sensor_data accel;
    struct bmi08_sensor_data gyro;
    for (uint16_t sample = 0; sample < ROBOT_IMU_CALIBRATION_SAMPLES; ++sample) {
        if (read_raw(&accel, &gyro) != ESP_OK) return ESP_FAIL;
        sums[0] += gyro.x;
        sums[1] += gyro.y;
        sums[2] += gyro.z;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    for (size_t i = 0; i < 3; ++i) {
        bias[i] = (float)sums[i] / (float)ROBOT_IMU_CALIBRATION_SAMPLES;
    }
    return ESP_OK;
}

static void mark_offline(bool initialization_error)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_snapshot.online = false;
    s_snapshot.calibrated = false;
    s_snapshot.sample_valid = false;
    if (initialization_error) ++s_snapshot.init_error_count;
    else ++s_snapshot.read_error_count;
    xSemaphoreGive(s_mutex);
}

static void bmi088_task(void *argument)
{
    (void)argument;
    float bias[3] = {0};
    for (;;) {
        if (initialize_device() != ESP_OK) {
            mark_offline(true);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (calibrate_gyro(bias) != ESP_OK) {
            mark_offline(false);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        TickType_t last_wake = xTaskGetTickCount();
        for (;;) {
            struct bmi08_sensor_data accel;
            struct bmi08_sensor_data gyro;
            if (read_raw(&accel, &gyro) != ESP_OK) {
                mark_offline(false);
                break;
            }
            publish_sample(&accel, &gyro, bias);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_IMU_SAMPLE_PERIOD_MS));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t bmi088_service_start(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) return ESP_ERR_NO_MEM;
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    return xTaskCreate(bmi088_task, "bmi088", 4096, NULL, 9, NULL) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

void bmi088_service_get_snapshot(bmi088_snapshot_t *snapshot)
{
    if (snapshot == NULL || s_mutex == NULL) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *snapshot = s_snapshot;
    xSemaphoreGive(s_mutex);
}
