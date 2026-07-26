#ifndef ROBOT_MOTOR_SERVICE_H
#define ROBOT_MOTOR_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "protocols/m0601c111_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_WHEEL_COUNT 2u
#define MOTOR_LEFT_INDEX  0u
#define MOTOR_RIGHT_INDEX 1u

typedef enum {
    MOTOR_ACTION_SET_RPM,
    MOTOR_ACTION_SET_CURRENT,
    MOTOR_ACTION_SET_POSITION,
    MOTOR_ACTION_STOP,
    MOTOR_ACTION_QUERY,
    MOTOR_ACTION_QUERY_UNIQUE_ID,
    MOTOR_ACTION_SET_ID,
    MOTOR_ACTION_SET_MODE,
    MOTOR_ACTION_SET_DUAL_RPM,
    MOTOR_ACTION_STOP_DUAL
} motor_action_t;

typedef struct {
    motor_action_t action;
    uint8_t id;
    int16_t target_value;
    int16_t right_target_value;
    uint8_t right_id;
    uint8_t accel;
    uint8_t brake;
    uint8_t expected_old_id;
    uint8_t new_id;
    uint8_t mode;
} motor_request_t;

typedef struct {
    uint32_t last_feedback_ms;
    uint32_t valid_feedback_count;
    uint16_t crc_error_count;
    uint16_t timeout_count;
    uint8_t motor_id;
    uint8_t mode;
    uint8_t state;
    uint8_t fault;
    int16_t target_value;
    int16_t actual_rpm;
    int16_t current_raw;
    uint16_t drive_position_raw;
    uint8_t query_position_u8;
    uint8_t stationary_samples;
    bool address_confirmed;
    bool control_active;
} motor_wheel_snapshot_t;

typedef struct {
    uint32_t uptime_ms;
    uint32_t last_control_ms;
    uint16_t watchdog_stop_count;
    bool control_active;
    bool dual_control_active;
    motor_wheel_snapshot_t wheel[MOTOR_WHEEL_COUNT];
    motor_wheel_snapshot_t selected;
} motor_chassis_snapshot_t;

/* Backward-compatible single-motor snapshot used by the legacy 0x90 heartbeat. */
typedef struct {
    uint32_t uptime_ms;
    uint32_t last_feedback_ms;
    uint32_t last_control_ms;
    uint32_t valid_feedback_count;
    uint16_t crc_error_count;
    uint16_t timeout_count;
    uint16_t watchdog_stop_count;
    uint8_t motor_id;
    uint8_t mode;
    uint8_t state;
    uint8_t fault;
    int16_t target_value;
    int16_t actual_rpm;
    int16_t current_raw;
    uint16_t drive_position_raw;
    uint8_t query_position_u8;
    uint8_t stationary_samples;
    bool address_confirmed;
    bool control_active;
} motor_snapshot_t;

typedef struct {
    m0601_status_t status;
    int16_t detail;
} motor_response_t;

esp_err_t motor_service_start(void);
motor_response_t motor_service_execute(const motor_request_t *request);
void motor_service_get_snapshot(motor_snapshot_t *snapshot);
void motor_service_get_chassis_snapshot(motor_chassis_snapshot_t *snapshot);
void motor_service_mark_control_received(void);
m0601_status_t motor_service_keepalive(uint8_t id);
m0601_status_t motor_service_dual_keepalive(uint8_t left_id, uint8_t right_id);
void motor_service_note_watchdog_stop(void);

#ifdef __cplusplus
}
#endif

#endif
