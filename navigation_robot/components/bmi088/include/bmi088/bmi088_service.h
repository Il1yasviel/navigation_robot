#ifndef ROBOT_BMI088_SERVICE_H
#define ROBOT_BMI088_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t timestamp_us;
    float accel_mps2[3];
    float gyro_rads[3];
    uint32_t sample_count;
    uint16_t read_error_count;
    uint16_t init_error_count;
    bool online;
    bool calibrated;
    bool sample_valid;
} bmi088_snapshot_t;

/* Starts a non-fatal background service. Missing hardware never blocks motors. */
esp_err_t bmi088_service_start(void);
void bmi088_service_get_snapshot(bmi088_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
