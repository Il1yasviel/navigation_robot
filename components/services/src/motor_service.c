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
static motor_snapshot_t s_snapshot;

static m0601_status_t transport_send(void *context,
                                     const uint8_t *data,
                                     size_t length,
                                     uint32_t timeout_ms)
{
    const esp_err_t status = esp32_rs485_send((esp32_rs485_t *)context,
                                               data,
                                               length,
                                               timeout_ms);
    if (status == ESP_OK) return M0601_OK;
    return status == ESP_ERR_TIMEOUT ? M0601_ERROR_TIMEOUT : M0601_ERROR_IO;
}

static m0601_status_t transport_receive(void *context,
                                        uint8_t *data,
                                        size_t length,
                                        uint32_t timeout_ms)
{
    const esp_err_t status = esp32_rs485_receive_exact((esp32_rs485_t *)context,
                                                        data,
                                                        length,
                                                        timeout_ms);
    if (status == ESP_OK) return M0601_OK;
    return status == ESP_ERR_TIMEOUT ? M0601_ERROR_TIMEOUT : M0601_ERROR_IO;
}

static void transport_delay(void *context, uint32_t delay_ms)
{
    (void)context;
    esp32_delay_ms(delay_ms);
}

static void record_error_locked(m0601_status_t status)
{
    if (status == M0601_ERROR_CRC) {
        ++s_snapshot.crc_error_count;
    } else if (status == M0601_ERROR_TIMEOUT) {
        ++s_snapshot.timeout_count;
    }
    if (status == M0601_ERROR_CRC || status == M0601_ERROR_TIMEOUT ||
        status == M0601_ERROR_IO) {
        s_snapshot.state = MOTOR_STATE_OFFLINE;
        s_snapshot.address_confirmed = false;
    }
}

static void update_drive_feedback(const m0601_drive_feedback_t *feedback,
                                  int16_t target_value,
                                  bool control_active)
{
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    s_snapshot.motor_id = feedback->id;
    s_snapshot.mode = feedback->mode;
    s_snapshot.target_value = target_value;
    s_snapshot.actual_rpm = feedback->speed_rpm;
    s_snapshot.current_raw = feedback->torque_current_raw;
    s_snapshot.drive_position_raw = feedback->position_raw;
    s_snapshot.fault = feedback->fault;
    s_snapshot.last_feedback_ms = esp32_time_millis();
    ++s_snapshot.valid_feedback_count;
    s_snapshot.address_confirmed = true;
    s_snapshot.control_active = control_active;
    s_snapshot.stationary_samples = abs(feedback->speed_rpm) < ROBOT_STATIONARY_RPM
                                        ? (uint8_t)(s_snapshot.stationary_samples +
                                            (s_snapshot.stationary_samples < UINT8_MAX))
                                        : 0;
    s_snapshot.state = feedback->fault != 0u
                           ? MOTOR_STATE_FAULT
                           : (control_active ? MOTOR_STATE_RUNNING : MOTOR_STATE_IDLE);
    xSemaphoreGive(s_snapshot_mutex);
}

static void update_query_feedback(const m0601_query_feedback_t *feedback)
{
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    s_snapshot.motor_id = feedback->id;
    s_snapshot.mode = feedback->mode;
    s_snapshot.actual_rpm = feedback->speed_rpm;
    s_snapshot.current_raw = feedback->torque_current_raw;
    s_snapshot.query_position_u8 = feedback->position_u8;
    s_snapshot.fault = feedback->fault;
    s_snapshot.last_feedback_ms = esp32_time_millis();
    ++s_snapshot.valid_feedback_count;
    s_snapshot.address_confirmed = true;
    s_snapshot.stationary_samples = abs(feedback->speed_rpm) < ROBOT_STATIONARY_RPM
                                        ? (uint8_t)(s_snapshot.stationary_samples +
                                            (s_snapshot.stationary_samples < UINT8_MAX))
                                        : 0;
    s_snapshot.state = feedback->fault != 0u
                           ? MOTOR_STATE_FAULT
                           : (s_snapshot.control_active ? MOTOR_STATE_RUNNING : MOTOR_STATE_IDLE);
    xSemaphoreGive(s_snapshot_mutex);
}

static void set_address_confirmed(bool confirmed)
{
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    s_snapshot.address_confirmed = confirmed;
    xSemaphoreGive(s_snapshot_mutex);
}

static m0601_status_t identify_unique_motor(int16_t *detail)
{
    m0601_drive_feedback_t identification;

    set_address_confirmed(false);
    m0601_status_t status = m0601_query_id(&s_motor, &identification);
    if (status != M0601_OK) {
        return status;
    }

    s_motor.default_id = identification.id;
    update_drive_feedback(&identification, 0, false);
    if (detail != NULL) {
        *detail = identification.id;
    }
    return M0601_OK;
}

static bool drive_preconditions_met(const motor_snapshot_t *snapshot,
                                    uint8_t id,
                                    uint8_t mode)
{
    return snapshot->address_confirmed && snapshot->motor_id == id &&
           snapshot->mode == mode;
}

static m0601_status_t stop_motor_safely(uint8_t id,
                                        uint8_t brake,
                                        m0601_drive_feedback_t *drive)
{
    motor_snapshot_t snapshot;
    motor_service_get_snapshot(&snapshot);
    if (!snapshot.address_confirmed || snapshot.motor_id != id) {
        return M0601_ERROR_FRAME;
    }

    m0601_status_t status;
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
                                     M0601_BRAKE_OFF, drive);
    } else {
        status = m0601_drive_speed(&s_motor, id, 0, M0601_ACCEL_DEFAULT,
                                   brake, drive);
    }
    if (status == M0601_OK) update_drive_feedback(drive, 0, false);
    return status;
}

static m0601_status_t process_request(const motor_request_t *request, int16_t *detail)
{
    m0601_status_t status;
    m0601_drive_feedback_t drive;
    m0601_query_feedback_t query;

    switch (request->action) {
    case MOTOR_ACTION_SET_RPM: {
        motor_snapshot_t snapshot;
        motor_service_get_snapshot(&snapshot);
        if (request->target_value < -CONFIG_ROBOT_TEST_MAX_RPM ||
            request->target_value > CONFIG_ROBOT_TEST_MAX_RPM) {
            return M0601_ERROR_RANGE;
        }
        if (!drive_preconditions_met(&snapshot, request->id, M0601_MODE_SPEED)) {
            return M0601_ERROR_FRAME;
        }
        status = m0601_drive_speed(&s_motor, request->id, request->target_value,
                                    request->accel, request->brake, &drive);
        if (status == M0601_OK) {
            update_drive_feedback(&drive, request->target_value,
                                  request->target_value != 0);
        }
        return status;
    }

    case MOTOR_ACTION_SET_CURRENT: {
        motor_snapshot_t snapshot;
        const int32_t limit_raw =
            ((int32_t)ROBOT_TEST_MAX_CURRENT_MA * M0601_CURRENT_RAW_MAX + 4000) / 8000;
        motor_service_get_snapshot(&snapshot);
        if ((int32_t)request->target_value < -limit_raw ||
            (int32_t)request->target_value > limit_raw) {
            return M0601_ERROR_RANGE;
        }
        if (!drive_preconditions_met(&snapshot, request->id, M0601_MODE_CURRENT)) {
            return M0601_ERROR_FRAME;
        }
        status = m0601_drive_current(&s_motor, request->id, request->target_value,
                                      request->accel, request->brake, &drive);
        if (status == M0601_OK) {
            update_drive_feedback(&drive, request->target_value,
                                  request->target_value != 0);
        }
        return status;
    }

    case MOTOR_ACTION_SET_POSITION: {
        motor_snapshot_t snapshot;
        motor_service_get_snapshot(&snapshot);
        if (request->target_value < 0) {
            return M0601_ERROR_RANGE;
        }
        if (!drive_preconditions_met(&snapshot, request->id, M0601_MODE_POSITION)) {
            return M0601_ERROR_FRAME;
        }
        status = m0601_drive_position(&s_motor, request->id,
                                       (uint16_t)request->target_value,
                                       request->accel, request->brake, &drive);
        if (status == M0601_OK) {
            update_drive_feedback(&drive, request->target_value, true);
        }
        return status;
    }

    case MOTOR_ACTION_STOP:
        status = stop_motor_safely(request->id, request->brake, &drive);
        if (status == M0601_OK) {
            if (request->brake == M0601_BRAKE_ON && drive.fault == 0u) {
                xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
                s_snapshot.state = MOTOR_STATE_ESTOP;
                xSemaphoreGive(s_snapshot_mutex);
            }
        }
        return status;

    case MOTOR_ACTION_QUERY:
        set_address_confirmed(false);
        status = m0601_query(&s_motor, request->id, &query);
        if (status == M0601_OK) {
            s_motor.default_id = request->id;
            update_query_feedback(&query);
        }
        return status;

    case MOTOR_ACTION_QUERY_UNIQUE_ID:
        return identify_unique_motor(detail);

    case MOTOR_ACTION_SET_MODE: {
        motor_snapshot_t snapshot;
        motor_service_get_snapshot(&snapshot);
        if (!snapshot.address_confirmed || snapshot.motor_id != request->id ||
            snapshot.control_active) {
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
        if (status == M0601_OK) {
            esp32_delay_ms(10);
            status = m0601_query(&s_motor, request->id, &query);
            if (status == M0601_OK) {
                update_query_feedback(&query);
                if (query.mode != request->mode) status = M0601_ERROR_FRAME;
            }
        }
        return status;
    }

    case MOTOR_ACTION_SET_ID: {
        motor_snapshot_t snapshot;
        motor_service_get_snapshot(&snapshot);
        if (!snapshot.address_confirmed || snapshot.motor_id != request->expected_old_id ||
            snapshot.control_active ||
            snapshot.stationary_samples < ROBOT_STATIONARY_SAMPLES) {
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

static void query_selected_motor(void)
{
    motor_snapshot_t snapshot;
    m0601_query_feedback_t feedback;
    motor_service_get_snapshot(&snapshot);
    const m0601_status_t status = m0601_query(&s_motor, snapshot.motor_id, &feedback);
    if (status == M0601_OK) {
        update_query_feedback(&feedback);
    } else {
        xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
        record_error_locked(status);
        xSemaphoreGive(s_snapshot_mutex);
    }
}

static void motor_task(void *argument)
{
    (void)argument;
    uint32_t next_query_ms = esp32_time_millis() + ROBOT_MOTOR_QUERY_MS;
    for (;;) {
        uint32_t now = esp32_time_millis();
        if (deadline_reached(now, next_query_ms)) {
            do {
                next_query_ms += ROBOT_MOTOR_QUERY_MS;
            } while (deadline_reached(now, next_query_ms));
            query_selected_motor();
            continue;
        }

        uint32_t wait_ms = next_query_ms - now;
        if (wait_ms > 20u) wait_ms = 20u;
        motor_request_t request;
        if (xQueueReceive(s_command_queue, &request, pdMS_TO_TICKS(wait_ms)) == pdTRUE) {
            motor_response_t response = {.detail = 0};
            response.status = process_request(&request, &response.detail);
            if (response.status != M0601_OK) {
                xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
                record_error_locked(response.status);
                xSemaphoreGive(s_snapshot_mutex);
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
        s_command_queue == NULL || s_response_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    const esp_err_t rs485_status = esp32_rs485_init(&s_bus);
    if (rs485_status != ESP_OK) {
        return rs485_status;
    }

    const m0601_transport_t transport = {
        .ctx = &s_bus,
        .send = transport_send,
        .recv = transport_receive,
        .delay_ms = transport_delay,
    };
    if (m0601_init(&s_motor, &transport, CONFIG_ROBOT_DEFAULT_MOTOR_ID,
                   CONFIG_ROBOT_MOTOR_TIMEOUT_MS) != M0601_OK) {
        return ESP_FAIL;
    }

    (void)memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.motor_id = CONFIG_ROBOT_DEFAULT_MOTOR_ID;
    s_snapshot.mode = M0601_MODE_SPEED;
    s_snapshot.state = MOTOR_STATE_OFFLINE;
    s_snapshot.last_control_ms = esp32_time_millis();

    return xTaskCreate(motor_task, "motor_bus", 4096, NULL, 12, NULL) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
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

void motor_service_get_snapshot(motor_snapshot_t *snapshot)
{
    if (snapshot == NULL || s_snapshot_mutex == NULL) return;
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    *snapshot = s_snapshot;
    snapshot->uptime_ms = esp32_time_millis();
    xSemaphoreGive(s_snapshot_mutex);
}

void motor_service_mark_control_received(void)
{
    if (s_snapshot_mutex == NULL) return;
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    s_snapshot.last_control_ms = esp32_time_millis();
    xSemaphoreGive(s_snapshot_mutex);
}

m0601_status_t motor_service_keepalive(uint8_t id)
{
    if (s_snapshot_mutex == NULL) return M0601_ERROR_NULL;

    m0601_status_t status = M0601_ERROR_FRAME;
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    if (s_snapshot.address_confirmed && s_snapshot.motor_id == id &&
        s_snapshot.control_active && s_snapshot.state != MOTOR_STATE_OFFLINE) {
        s_snapshot.last_control_ms = esp32_time_millis();
        status = M0601_OK;
    }
    xSemaphoreGive(s_snapshot_mutex);
    return status;
}

void motor_service_note_watchdog_stop(void)
{
    if (s_snapshot_mutex == NULL) return;
    xSemaphoreTake(s_snapshot_mutex, portMAX_DELAY);
    ++s_snapshot.watchdog_stop_count;
    s_snapshot.state = MOTOR_STATE_ESTOP;
    xSemaphoreGive(s_snapshot_mutex);
}
