#include "services/motor_service.h"

#include <stdlib.h>
#include <string.h>

#include "config/robot_config.h"
#include "esp32_drivers/rs485_uart.h"
#include "esp32_drivers/system_time.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "protocols/host_messages.h"

static QueueHandle_t s_command_queue;
static QueueHandle_t s_response_queue;
static SemaphoreHandle_t s_submit_mutex;
static SemaphoreHandle_t s_snapshot_mutex;
static esp32_rs485_t s_bus;
static m0601_motor_t s_motor;
static motor_chassis_snapshot_t s_chassis;

static m0601_status_t transport_send(void *context, const uint8_t *data,
                                     size_t length, uint32_t timeout_ms)
{
    const esp_err_t status = esp32_rs485_send((esp32_rs485_t *)context,
                                               data, length, timeout_ms);
    if (status == ESP_OK) return M0601_OK;
    return status == ESP_ERR_TIMEOUT ? M0601_ERROR_TIMEOUT : M0601_ERROR_IO;
}

static m0601_status_t transport_receive(void *context, uint8_t *data,
                                        size_t length, uint32_t timeout_ms)
{
    const esp_err_t status = esp32_rs485_receive_exact((esp32_rs485_t *)context,
                                                        data, length, timeout_ms);
    if (status == ESP_OK) return M0601_OK;
    return status == ESP_ERR_TIMEOUT ? M0601_ERROR_TIMEOUT : M0601_ERROR_IO;
}

static void transport_delay(void *context, uint32_t delay_ms)
{
    (void)context;
    esp32_delay_ms(delay_ms);
}

static int wheel_index_for_id(uint8_t id)
{
    if (id == (uint8_t)CONFIG_ROBOT_LEFT_MOTOR_ID) return (int)MOTOR_LEFT_INDEX;
    if (id == (uint8_t)CONFIG_ROBOT_RIGHT_MOTOR_ID) return (int)MOTOR_RIGHT_INDEX;
    return -1;
}

static int wheel_direction_for_id(uint8_t id)
{
    return id == (uint8_t)CONFIG_ROBOT_RIGHT_MOTOR_ID
               ? ROBOT_RIGHT_DIRECTION : ROBOT_LEFT_DIRECTION;
}

static void record_error_in_snapshot(motor_wheel_snapshot_t *snapshot,
                                     m0601_status_t status)
{
    if (status == M0601_ERROR_CRC) ++snapshot->crc_error_count;
    if (status == M0601_ERROR_TIMEOUT) ++snapshot->timeout_count;
    if (status == M0601_ERROR_CRC || status == M0601_ERROR_TIMEOUT ||
        status == M0601_ERROR_IO) {
        snapshot->state = MOTOR_STATE_OFFLINE;
        snapshot->address_confirmed = false;
        snapshot->control_active = false;
    }
}

static void record_error(uint8_t id, m0601_status_t status)
{
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    const int index = wheel_index_for_id(id);
    if (index >= 0) record_error_in_snapshot(&s_chassis.wheel[index], status);
    if (s_chassis.selected.motor_id == id) {
        record_error_in_snapshot(&s_chassis.selected, status);
    }
    s_chassis.control_active = s_chassis.wheel[MOTOR_LEFT_INDEX].control_active ||
                               s_chassis.wheel[MOTOR_RIGHT_INDEX].control_active ||
                               s_chassis.selected.control_active;
    xSemaphoreGive(s_snapshot_mutex);
}

static void apply_feedback(motor_wheel_snapshot_t *snapshot, uint8_t id,
                           uint8_t mode, int16_t target_value,
                           int16_t actual_rpm, int16_t current_raw,
                           uint16_t drive_position, uint8_t query_position,
                           uint8_t fault, bool control_active)
{
    snapshot->motor_id = id;
    snapshot->mode = mode;
    snapshot->target_value = target_value;
    snapshot->actual_rpm = actual_rpm;
    snapshot->current_raw = current_raw;
    if (drive_position != UINT16_MAX) snapshot->drive_position_raw = drive_position;
    if (query_position != UINT8_MAX) snapshot->query_position_u8 = query_position;
    snapshot->fault = fault;
    snapshot->last_feedback_ms = esp32_time_millis();
    ++snapshot->valid_feedback_count;
    snapshot->address_confirmed = true;
    snapshot->control_active = control_active;
    snapshot->stationary_samples = abs(actual_rpm) < ROBOT_STATIONARY_RPM
                                       ? (uint8_t)(snapshot->stationary_samples +
                                           (snapshot->stationary_samples < UINT8_MAX))
                                       : 0u;
    snapshot->state = fault != 0u ? MOTOR_STATE_FAULT
                                  : (control_active ? MOTOR_STATE_RUNNING
                                                    : MOTOR_STATE_IDLE);
}

static void update_drive_feedback(const m0601_drive_feedback_t *feedback,
                                  int16_t logical_target, bool control_active)
{
    const int direction = wheel_direction_for_id(feedback->id);
    const int16_t logical_actual = (int16_t)(feedback->speed_rpm * direction);
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    const int index = wheel_index_for_id(feedback->id);
    if (index >= 0) {
        apply_feedback(&s_chassis.wheel[index], feedback->id, feedback->mode,
                       logical_target, logical_actual,
                       feedback->torque_current_raw, feedback->position_raw,
                       UINT8_MAX, feedback->fault, control_active);
    }
    if (s_chassis.selected.motor_id == feedback->id) {
        apply_feedback(&s_chassis.selected, feedback->id, feedback->mode,
                       logical_target, logical_actual,
                       feedback->torque_current_raw, feedback->position_raw,
                       UINT8_MAX, feedback->fault, control_active);
    }
    s_chassis.control_active = s_chassis.wheel[MOTOR_LEFT_INDEX].control_active ||
                               s_chassis.wheel[MOTOR_RIGHT_INDEX].control_active ||
                               s_chassis.selected.control_active;
    xSemaphoreGive(s_snapshot_mutex);
}

static void update_query_feedback(const m0601_query_feedback_t *feedback)
{
    const int direction = wheel_direction_for_id(feedback->id);
    const int16_t logical_actual = (int16_t)(feedback->speed_rpm * direction);
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    const int index = wheel_index_for_id(feedback->id);
    if (index >= 0) {
        motor_wheel_snapshot_t *wheel = &s_chassis.wheel[index];
        apply_feedback(wheel, feedback->id, feedback->mode, wheel->target_value,
                       logical_actual, feedback->torque_current_raw, UINT16_MAX,
                       feedback->position_u8, feedback->fault,
                       wheel->control_active);
    }
    if (s_chassis.selected.motor_id == feedback->id) {
        motor_wheel_snapshot_t *selected = &s_chassis.selected;
        apply_feedback(selected, feedback->id, feedback->mode,
                       selected->target_value, logical_actual,
                       feedback->torque_current_raw, UINT16_MAX,
                       feedback->position_u8, feedback->fault,
                       selected->control_active);
    }
    xSemaphoreGive(s_snapshot_mutex);
}

static void select_motor(uint8_t id, bool confirmed)
{
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    const int index = wheel_index_for_id(id);
    if (index >= 0) s_chassis.selected = s_chassis.wheel[index];
    s_chassis.selected.motor_id = id;
    s_chassis.selected.address_confirmed = confirmed;
    xSemaphoreGive(s_snapshot_mutex);
}

static m0601_status_t identify_unique_motor(int16_t *detail)
{
    m0601_drive_feedback_t feedback;
    select_motor(s_chassis.selected.motor_id, false);
    m0601_status_t status = m0601_query_id(&s_motor, &feedback);
    if (status != M0601_OK) return status;
    s_motor.default_id = feedback.id;
    select_motor(feedback.id, true);
    update_drive_feedback(&feedback, 0, false);
    if (detail != NULL) *detail = feedback.id;
    return M0601_OK;
}

static bool snapshot_is_fresh(const motor_wheel_snapshot_t *snapshot,
                              uint32_t now)
{
    return snapshot->address_confirmed &&
           (now - snapshot->last_feedback_ms) <= ROBOT_MOTOR_FEEDBACK_FRESH_MS;
}

static bool drive_preconditions_met(const motor_wheel_snapshot_t *snapshot,
                                    uint8_t id, uint8_t mode)
{
    return snapshot->motor_id == id && snapshot->mode == mode &&
           snapshot->fault == 0u && snapshot_is_fresh(snapshot, esp32_time_millis());
}

static m0601_status_t stop_motor_safely(uint8_t id, uint8_t brake)
{
    motor_chassis_snapshot_t chassis;
    motor_service_get_chassis_snapshot(&chassis);
    motor_wheel_snapshot_t snapshot = chassis.selected;
    const int index = wheel_index_for_id(id);
    if (index >= 0) snapshot = chassis.wheel[index];
    if (!snapshot.address_confirmed || snapshot.motor_id != id) {
        return M0601_ERROR_FRAME;
    }

    m0601_status_t status;
    m0601_drive_feedback_t drive;
    if (snapshot.mode == M0601_MODE_POSITION) {
        m0601_query_feedback_t confirmation;
        status = m0601_set_mode(&s_motor, id, M0601_MODE_SPEED);
        if (status != M0601_OK) return status;
        esp32_delay_ms(10);
        status = m0601_query(&s_motor, id, &confirmation);
        if (status != M0601_OK) return status;
        update_query_feedback(&confirmation);
        if (confirmation.mode != M0601_MODE_SPEED) return M0601_ERROR_FRAME;
        snapshot.mode = M0601_MODE_SPEED;
    }

    if (snapshot.mode == M0601_MODE_CURRENT) {
        status = m0601_drive_current(&s_motor, id, 0, M0601_ACCEL_DEFAULT,
                                     M0601_BRAKE_OFF, &drive);
    } else {
        status = m0601_drive_speed(&s_motor, id, 0, M0601_ACCEL_DEFAULT,
                                   brake, &drive);
    }
    if (status == M0601_OK) update_drive_feedback(&drive, 0, false);
    return status;
}

static m0601_status_t stop_dual_safely(uint8_t brake)
{
    motor_chassis_snapshot_t before;
    motor_service_get_chassis_snapshot(&before);
    const m0601_status_t left = stop_motor_safely(
        (uint8_t)CONFIG_ROBOT_LEFT_MOTOR_ID, brake);
    const m0601_status_t right = stop_motor_safely(
        (uint8_t)CONFIG_ROBOT_RIGHT_MOTOR_ID, brake);
    m0601_status_t selected = M0601_OK;
    if (before.selected.control_active &&
        before.selected.motor_id != (uint8_t)CONFIG_ROBOT_LEFT_MOTOR_ID &&
        before.selected.motor_id != (uint8_t)CONFIG_ROBOT_RIGHT_MOTOR_ID) {
        selected = stop_motor_safely(before.selected.motor_id, brake);
    }
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    s_chassis.dual_control_active = false;
    s_chassis.control_active = false;
    s_chassis.wheel[MOTOR_LEFT_INDEX].control_active = false;
    s_chassis.wheel[MOTOR_RIGHT_INDEX].control_active = false;
    s_chassis.selected.control_active = false;
    xSemaphoreGive(s_snapshot_mutex);
    if (left != M0601_OK) return left;
    return right != M0601_OK ? right : selected;
}

static m0601_status_t drive_dual(const motor_request_t *request)
{
    if (request->id != (uint8_t)CONFIG_ROBOT_LEFT_MOTOR_ID ||
        request->right_id != (uint8_t)CONFIG_ROBOT_RIGHT_MOTOR_ID ||
        request->target_value < -CONFIG_ROBOT_TEST_MAX_RPM ||
        request->target_value > CONFIG_ROBOT_TEST_MAX_RPM ||
        request->right_target_value < -CONFIG_ROBOT_TEST_MAX_RPM ||
        request->right_target_value > CONFIG_ROBOT_TEST_MAX_RPM) {
        return M0601_ERROR_RANGE;
    }

    motor_chassis_snapshot_t chassis;
    motor_service_get_chassis_snapshot(&chassis);
    if (!drive_preconditions_met(&chassis.wheel[MOTOR_LEFT_INDEX], request->id,
                                 M0601_MODE_SPEED) ||
        !drive_preconditions_met(&chassis.wheel[MOTOR_RIGHT_INDEX], request->right_id,
                                 M0601_MODE_SPEED)) {
        return M0601_ERROR_FRAME;
    }

    m0601_drive_feedback_t feedback;
    m0601_status_t status = m0601_drive_speed(
        &s_motor, request->id,
        (int16_t)(request->target_value * ROBOT_LEFT_DIRECTION),
        request->accel, request->brake, &feedback);
    if (status != M0601_OK) return status;
    update_drive_feedback(&feedback, request->target_value,
                          request->target_value != 0 || request->right_target_value != 0);

    status = m0601_drive_speed(
        &s_motor, request->right_id,
        (int16_t)(request->right_target_value * ROBOT_RIGHT_DIRECTION),
        request->accel, request->brake, &feedback);
    if (status != M0601_OK) {
        (void)stop_dual_safely(M0601_BRAKE_ON);
        return status;
    }
    const bool active = request->target_value != 0 || request->right_target_value != 0;
    update_drive_feedback(&feedback, request->right_target_value, active);
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    s_chassis.dual_control_active = active;
    s_chassis.control_active = active;
    s_chassis.last_control_ms = esp32_time_millis();
    s_chassis.wheel[MOTOR_LEFT_INDEX].control_active = active;
    s_chassis.wheel[MOTOR_RIGHT_INDEX].control_active = active;
    xSemaphoreGive(s_snapshot_mutex);
    return M0601_OK;
}

static m0601_status_t process_request(const motor_request_t *request, int16_t *detail)
{
    m0601_status_t status;
    m0601_drive_feedback_t drive;
    m0601_query_feedback_t query;

    switch (request->action) {
    case MOTOR_ACTION_SET_DUAL_RPM:
        return drive_dual(request);
    case MOTOR_ACTION_STOP_DUAL:
        if (request->id != (uint8_t)CONFIG_ROBOT_LEFT_MOTOR_ID ||
            request->right_id != (uint8_t)CONFIG_ROBOT_RIGHT_MOTOR_ID) {
            return M0601_ERROR_RANGE;
        }
        return stop_dual_safely(request->brake);
    case MOTOR_ACTION_SET_RPM: {
        motor_chassis_snapshot_t chassis;
        motor_service_get_chassis_snapshot(&chassis);
        if (request->target_value < -CONFIG_ROBOT_TEST_MAX_RPM ||
            request->target_value > CONFIG_ROBOT_TEST_MAX_RPM) return M0601_ERROR_RANGE;
        if (!drive_preconditions_met(&chassis.selected, request->id,
                                     M0601_MODE_SPEED)) return M0601_ERROR_FRAME;
        const int wire_target = request->target_value * wheel_direction_for_id(request->id);
        status = m0601_drive_speed(&s_motor, request->id, (int16_t)wire_target,
                                   request->accel, request->brake, &drive);
        if (status == M0601_OK) update_drive_feedback(
            &drive, request->target_value, request->target_value != 0);
        return status;
    }
    case MOTOR_ACTION_SET_CURRENT: {
        motor_chassis_snapshot_t chassis;
        const int32_t limit_raw = ((int32_t)ROBOT_TEST_MAX_CURRENT_MA *
            M0601_CURRENT_RAW_MAX + 4000) / 8000;
        motor_service_get_chassis_snapshot(&chassis);
        if ((int32_t)request->target_value < -limit_raw ||
            (int32_t)request->target_value > limit_raw) return M0601_ERROR_RANGE;
        if (!drive_preconditions_met(&chassis.selected, request->id,
                                     M0601_MODE_CURRENT)) return M0601_ERROR_FRAME;
        status = m0601_drive_current(&s_motor, request->id, request->target_value,
                                     request->accel, request->brake, &drive);
        if (status == M0601_OK) update_drive_feedback(
            &drive, request->target_value, request->target_value != 0);
        return status;
    }
    case MOTOR_ACTION_SET_POSITION: {
        motor_chassis_snapshot_t chassis;
        motor_service_get_chassis_snapshot(&chassis);
        if (request->target_value < 0) return M0601_ERROR_RANGE;
        if (!drive_preconditions_met(&chassis.selected, request->id,
                                     M0601_MODE_POSITION)) return M0601_ERROR_FRAME;
        status = m0601_drive_position(&s_motor, request->id,
                                      (uint16_t)request->target_value,
                                      request->accel, request->brake, &drive);
        if (status == M0601_OK) update_drive_feedback(
            &drive, request->target_value, true);
        return status;
    }
    case MOTOR_ACTION_STOP:
        status = stop_motor_safely(request->id, request->brake);
        if (status == M0601_OK && request->brake == M0601_BRAKE_ON) {
            xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
            s_chassis.selected.state = MOTOR_STATE_ESTOP;
            xSemaphoreGive(s_snapshot_mutex);
        }
        return status;
    case MOTOR_ACTION_QUERY:
        select_motor(request->id, false);
        status = m0601_query(&s_motor, request->id, &query);
        if (status == M0601_OK) {
            s_motor.default_id = request->id;
            update_query_feedback(&query);
        }
        return status;
    case MOTOR_ACTION_QUERY_UNIQUE_ID:
        return identify_unique_motor(detail);
    case MOTOR_ACTION_SET_MODE: {
        motor_chassis_snapshot_t chassis;
        motor_service_get_chassis_snapshot(&chassis);
        if (!chassis.selected.address_confirmed ||
            chassis.selected.motor_id != request->id || chassis.control_active) {
            return M0601_ERROR_FRAME;
        }
        status = m0601_query(&s_motor, request->id, &query);
        if (status != M0601_OK) return status;
        update_query_feedback(&query);
        if (request->mode == M0601_MODE_POSITION &&
            abs(query.speed_rpm) >= ROBOT_POSITION_MODE_MAX_RPM) {
            return M0601_ERROR_FRAME;
        }
        status = m0601_set_mode(&s_motor, request->id, request->mode);
        if (status != M0601_OK) return status;
        esp32_delay_ms(10);
        status = m0601_query(&s_motor, request->id, &query);
        if (status == M0601_OK) {
            update_query_feedback(&query);
            if (query.mode != request->mode) status = M0601_ERROR_FRAME;
        }
        return status;
    }
    case MOTOR_ACTION_SET_ID: {
        motor_chassis_snapshot_t chassis;
        motor_service_get_chassis_snapshot(&chassis);
        if (!chassis.selected.address_confirmed ||
            chassis.selected.motor_id != request->expected_old_id ||
            chassis.control_active ||
            chassis.selected.stationary_samples < ROBOT_STATIONARY_SAMPLES) {
            return M0601_ERROR_FRAME;
        }
        int16_t detected_id = -1;
        status = identify_unique_motor(&detected_id);
        if (status != M0601_OK || detected_id != request->expected_old_id) {
            return status == M0601_OK ? M0601_ERROR_FRAME : status;
        }
        status = m0601_set_id(&s_motor, request->new_id);
        if (status != M0601_OK) return status;
        esp32_delay_ms(100);
        status = identify_unique_motor(&detected_id);
        if (status != M0601_OK || detected_id != request->new_id) {
            return status == M0601_OK ? M0601_ERROR_FRAME : status;
        }
        if (detail != NULL) *detail = detected_id;
        return M0601_OK;
    }
    default:
        return M0601_ERROR_FRAME;
    }
}

static bool deadline_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static void query_wheel(uint8_t id)
{
    m0601_query_feedback_t feedback;
    const m0601_status_t status = m0601_query(&s_motor, id, &feedback);
    if (status == M0601_OK) update_query_feedback(&feedback);
    else record_error(id, status);
}

static void motor_task(void *argument)
{
    (void)argument;
    uint32_t next_query_ms = esp32_time_millis();
    uint8_t next_wheel = MOTOR_LEFT_INDEX;
    for (;;) {
        const uint32_t now = esp32_time_millis();
        if (deadline_reached(now, next_query_ms)) {
            do { next_query_ms += ROBOT_MOTOR_QUERY_MS; }
            while (deadline_reached(now, next_query_ms));
            query_wheel(next_wheel == MOTOR_LEFT_INDEX
                            ? (uint8_t)CONFIG_ROBOT_LEFT_MOTOR_ID
                            : (uint8_t)CONFIG_ROBOT_RIGHT_MOTOR_ID);
            next_wheel ^= 1u;
            continue;
        }
        uint32_t wait_ms = next_query_ms - now;
        if (wait_ms > 20u) wait_ms = 20u;
        motor_request_t request;
        if (xQueueReceive(s_command_queue, &request, pdMS_TO_TICKS(wait_ms)) == pdTRUE) {
            motor_response_t response = {.detail = 0};
            response.status = process_request(&request, &response.detail);
            if (response.status != M0601_OK) {
                uint8_t id = request.id;
                if (request.action == MOTOR_ACTION_QUERY_UNIQUE_ID) {
                    id = s_chassis.selected.motor_id;
                }
                record_error(id, response.status);
                if (request.action == MOTOR_ACTION_SET_DUAL_RPM ||
                    request.action == MOTOR_ACTION_STOP_DUAL) {
                    record_error(request.right_id, response.status);
                }
            }
            xQueueSend(s_response_queue, &response, portMAX_DELAY);
        }
    }
}

esp_err_t motor_service_start(void)
{
    s_snapshot_mutex = xSemaphoreCreateMutex();
    s_submit_mutex = xSemaphoreCreateMutex();
    s_command_queue = xQueueCreate(ROBOT_MOTOR_QUEUE_LENGTH, sizeof(motor_request_t));
    s_response_queue = xQueueCreate(1, sizeof(motor_response_t));
    if (s_snapshot_mutex == NULL || s_submit_mutex == NULL ||
        s_command_queue == NULL || s_response_queue == NULL) return ESP_ERR_NO_MEM;
    const esp_err_t bus_status = esp32_rs485_init(&s_bus);
    if (bus_status != ESP_OK) return bus_status;
    const m0601_transport_t transport = {
        .ctx = &s_bus, .send = transport_send,
        .recv = transport_receive, .delay_ms = transport_delay,
    };
    if (m0601_init(&s_motor, &transport, CONFIG_ROBOT_LEFT_MOTOR_ID,
                   CONFIG_ROBOT_MOTOR_TIMEOUT_MS) != M0601_OK) return ESP_FAIL;

    memset(&s_chassis, 0, sizeof(s_chassis));
    s_chassis.wheel[MOTOR_LEFT_INDEX].motor_id = CONFIG_ROBOT_LEFT_MOTOR_ID;
    s_chassis.wheel[MOTOR_RIGHT_INDEX].motor_id = CONFIG_ROBOT_RIGHT_MOTOR_ID;
    for (size_t i = 0; i < MOTOR_WHEEL_COUNT; ++i) {
        s_chassis.wheel[i].mode = M0601_MODE_SPEED;
        s_chassis.wheel[i].state = MOTOR_STATE_OFFLINE;
    }
    s_chassis.selected = s_chassis.wheel[MOTOR_LEFT_INDEX];
    s_chassis.last_control_ms = esp32_time_millis();
    return xTaskCreate(motor_task, "motor_bus", 5120, NULL, 12, NULL) == pdPASS
               ? ESP_OK : ESP_ERR_NO_MEM;
}

motor_response_t motor_service_execute(const motor_request_t *request)
{
    motor_response_t response = {.status = M0601_ERROR_NULL, .detail = 0};
    if (request == NULL || s_command_queue == NULL) return response;
    xSemaphoreTake(s_submit_mutex, portMAX_DELAY);
    xQueueReset(s_response_queue);
    if (xQueueSend(s_command_queue, request, portMAX_DELAY) == pdTRUE) {
        (void)xQueueReceive(s_response_queue, &response, portMAX_DELAY);
    } else {
        response.status = M0601_ERROR_IO;
    }
    xSemaphoreGive(s_submit_mutex);
    return response;
}

void motor_service_get_chassis_snapshot(motor_chassis_snapshot_t *snapshot)
{
    if (snapshot == NULL || s_snapshot_mutex == NULL) return;
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    *snapshot = s_chassis;
    snapshot->uptime_ms = esp32_time_millis();
    xSemaphoreGive(s_snapshot_mutex);
}

void motor_service_get_snapshot(motor_snapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    motor_chassis_snapshot_t chassis;
    motor_service_get_chassis_snapshot(&chassis);
    const motor_wheel_snapshot_t *selected = &chassis.selected;
    snapshot->uptime_ms = chassis.uptime_ms;
    snapshot->last_feedback_ms = selected->last_feedback_ms;
    snapshot->last_control_ms = chassis.last_control_ms;
    snapshot->valid_feedback_count = selected->valid_feedback_count;
    snapshot->crc_error_count = selected->crc_error_count;
    snapshot->timeout_count = selected->timeout_count;
    snapshot->watchdog_stop_count = chassis.watchdog_stop_count;
    snapshot->motor_id = selected->motor_id;
    snapshot->mode = selected->mode;
    snapshot->state = selected->state;
    snapshot->fault = selected->fault;
    snapshot->target_value = selected->target_value;
    snapshot->actual_rpm = selected->actual_rpm;
    snapshot->current_raw = selected->current_raw;
    snapshot->drive_position_raw = selected->drive_position_raw;
    snapshot->query_position_u8 = selected->query_position_u8;
    snapshot->stationary_samples = selected->stationary_samples;
    snapshot->address_confirmed = selected->address_confirmed;
    snapshot->control_active = selected->control_active;
}

void motor_service_mark_control_received(void)
{
    if (s_snapshot_mutex == NULL) return;
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    s_chassis.last_control_ms = esp32_time_millis();
    xSemaphoreGive(s_snapshot_mutex);
}

m0601_status_t motor_service_keepalive(uint8_t id)
{
    if (s_snapshot_mutex == NULL) return M0601_ERROR_NULL;
    m0601_status_t status = M0601_ERROR_FRAME;
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    if (!s_chassis.dual_control_active && s_chassis.selected.address_confirmed &&
        s_chassis.selected.motor_id == id && s_chassis.selected.control_active &&
        s_chassis.selected.state != MOTOR_STATE_OFFLINE) {
        s_chassis.last_control_ms = esp32_time_millis();
        status = M0601_OK;
    }
    xSemaphoreGive(s_snapshot_mutex);
    return status;
}

m0601_status_t motor_service_dual_keepalive(uint8_t left_id, uint8_t right_id)
{
    if (s_snapshot_mutex == NULL) return M0601_ERROR_NULL;
    m0601_status_t status = M0601_ERROR_FRAME;
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    if (left_id == (uint8_t)CONFIG_ROBOT_LEFT_MOTOR_ID &&
        right_id == (uint8_t)CONFIG_ROBOT_RIGHT_MOTOR_ID &&
        s_chassis.dual_control_active &&
        s_chassis.wheel[MOTOR_LEFT_INDEX].address_confirmed &&
        s_chassis.wheel[MOTOR_RIGHT_INDEX].address_confirmed &&
        s_chassis.wheel[MOTOR_LEFT_INDEX].state != MOTOR_STATE_OFFLINE &&
        s_chassis.wheel[MOTOR_RIGHT_INDEX].state != MOTOR_STATE_OFFLINE) {
        s_chassis.last_control_ms = esp32_time_millis();
        status = M0601_OK;
    }
    xSemaphoreGive(s_snapshot_mutex);
    return status;
}

void motor_service_note_watchdog_stop(void)
{
    if (s_snapshot_mutex == NULL) return;
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    ++s_chassis.watchdog_stop_count;
    s_chassis.control_active = false;
    s_chassis.dual_control_active = false;
    s_chassis.wheel[MOTOR_LEFT_INDEX].state = MOTOR_STATE_ESTOP;
    s_chassis.wheel[MOTOR_RIGHT_INDEX].state = MOTOR_STATE_ESTOP;
    s_chassis.wheel[MOTOR_LEFT_INDEX].control_active = false;
    s_chassis.wheel[MOTOR_RIGHT_INDEX].control_active = false;
    s_chassis.selected.control_active = false;
    xSemaphoreGive(s_snapshot_mutex);
}
