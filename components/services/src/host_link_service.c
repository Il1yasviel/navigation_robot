#include "services/host_link_service.h"

#include <string.h>

#include "algorithms/joystick_mapping.h"
#include "bmi088/bmi088_service.h"
#include "common/byte_order.h"
#include "config/robot_config.h"
#include "esp32_drivers/host_uart.h"
#include "esp32_drivers/wifi_tcp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "protocols/host_frame.h"
#include "protocols/host_messages.h"
#include "protocols/m0601c111_motor.h"
#include "services/motor_service.h"

typedef enum {
    HOST_SOURCE_NONE = HOST_CONTROL_OWNER_NONE,
    HOST_SOURCE_UART = HOST_CONTROL_OWNER_UART,
    HOST_SOURCE_TCP = HOST_CONTROL_OWNER_TCP,
} host_source_t;

typedef struct {
    host_source_t source;
    host_stream_parser_t parser;
} host_source_context_t;

static SemaphoreHandle_t s_uart_write_mutex;
static SemaphoreHandle_t s_owner_mutex;
static host_source_t s_control_owner = HOST_SOURCE_NONE;
static uint8_t s_legacy_sequence;
static uint8_t s_chassis_sequence;
static uint8_t s_imu_sequence;
static host_source_context_t s_uart_context = {.source = HOST_SOURCE_UART};
static host_source_context_t s_tcp_context = {.source = HOST_SOURCE_TCP};

static host_status_t map_motor_status(m0601_status_t status)
{
    switch (status) {
    case M0601_OK: return HOST_STATUS_OK;
    case M0601_ERROR_RANGE: return HOST_STATUS_RANGE;
    case M0601_ERROR_TIMEOUT: return HOST_STATUS_MOTOR_TIMEOUT;
    case M0601_ERROR_CRC: return HOST_STATUS_MOTOR_CRC;
    case M0601_ERROR_FRAME: return HOST_STATUS_PRECONDITION;
    case M0601_ERROR_IO: return HOST_STATUS_IO;
    default: return HOST_STATUS_IO;
    }
}

static void write_encoded(host_source_t source, const uint8_t *data, size_t length)
{
    if (source == HOST_SOURCE_UART) {
        xSemaphoreTake(s_uart_write_mutex, portMAX_DELAY);
        (void)esp32_host_uart_write(data, length, 100);
        xSemaphoreGive(s_uart_write_mutex);
    } else if (source == HOST_SOURCE_TCP && esp32_wifi_tcp_connected()) {
        (void)esp32_wifi_tcp_write(data, length, 100);
    }
}

static void send_frame_to(host_source_t source, uint8_t type, uint8_t sequence,
                          const uint8_t *payload, uint16_t payload_length)
{
    uint8_t encoded[HOST_FRAME_MAX_SIZE];
    const size_t length = host_frame_encode(type, sequence, 0u, payload,
                                             payload_length, encoded,
                                             sizeof(encoded));
    if (length != 0u) write_encoded(source, encoded, length);
}

static void broadcast_frame(uint8_t type, uint8_t sequence,
                            const uint8_t *payload, uint16_t payload_length)
{
    uint8_t encoded[HOST_FRAME_MAX_SIZE];
    const size_t length = host_frame_encode(type, sequence, 0u, payload,
                                             payload_length, encoded,
                                             sizeof(encoded));
    if (length == 0u) return;
    write_encoded(HOST_SOURCE_UART, encoded, length);
    write_encoded(HOST_SOURCE_TCP, encoded, length);
}

static void send_ack(host_source_t source, const host_frame_t *request,
                     host_status_t status, int16_t detail)
{
    uint8_t payload[4];
    payload[0] = request->type;
    payload[1] = (uint8_t)status;
    robot_write_i16_le(&payload[2], detail);
    send_frame_to(source, HOST_MSG_ACK, request->sequence, payload, sizeof(payload));
}

static bool valid_motor_id(uint8_t id)
{
    return id != M0601_ID_QUERY_ADDRESS;
}

static host_source_t control_owner(void)
{
    xSemaphoreTake(s_owner_mutex, portMAX_DELAY);
    const host_source_t owner = s_control_owner;
    xSemaphoreGive(s_owner_mutex);
    return owner;
}

static bool claim_control(host_source_t source, bool *newly_claimed)
{
    bool accepted = false;
    *newly_claimed = false;
    xSemaphoreTake(s_owner_mutex, portMAX_DELAY);
    if (s_control_owner == HOST_SOURCE_NONE) {
        s_control_owner = source;
        *newly_claimed = true;
        accepted = true;
    } else if (s_control_owner == source) {
        accepted = true;
    }
    xSemaphoreGive(s_owner_mutex);
    return accepted;
}

static void release_control(host_source_t source)
{
    xSemaphoreTake(s_owner_mutex, portMAX_DELAY);
    if (source == HOST_SOURCE_NONE || s_control_owner == source) {
        s_control_owner = HOST_SOURCE_NONE;
    }
    xSemaphoreGive(s_owner_mutex);
}

static motor_response_t execute_control(const motor_request_t *request)
{
    motor_response_t response = motor_service_execute(request);
    if (response.status == M0601_OK) motor_service_mark_control_received();
    return response;
}

static motor_response_t execute_owned_control(host_source_t source,
                                              const motor_request_t *request,
                                              bool active,
                                              host_status_t *host_status)
{
    motor_response_t response = {.status = M0601_ERROR_FRAME, .detail = 0};
    bool newly_claimed;
    if (!claim_control(source, &newly_claimed)) {
        *host_status = HOST_STATUS_BUSY;
        return response;
    }
    response = execute_control(request);
    *host_status = map_motor_status(response.status);
    if (response.status != M0601_OK && newly_claimed) release_control(source);
    if (response.status == M0601_OK && !active) release_control(source);
    return response;
}

static void dispatch_host_frame(void *context, const host_frame_t *frame)
{
    const host_source_t source = ((host_source_context_t *)context)->source;
    motor_request_t request = {0};
    motor_response_t response = {.status = M0601_OK, .detail = 0};
    host_status_t status = HOST_STATUS_OK;

    switch (frame->type) {
    case HOST_MSG_HELLO:
        if (frame->payload_length != 0u) status = HOST_STATUS_BAD_LENGTH;
        break;
    case HOST_MSG_SET_SINGLE_RPM:
    case HOST_MSG_SET_CURRENT:
    case HOST_MSG_SET_POSITION: {
        if (frame->payload_length != 5u) {
            status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.id = frame->payload[0];
        request.target_value = robot_read_i16_le(&frame->payload[1]);
        request.accel = frame->payload[3];
        request.brake = frame->payload[4];
        if (!valid_motor_id(request.id)) {
            status = HOST_STATUS_RANGE;
            break;
        }
        if (frame->type == HOST_MSG_SET_SINGLE_RPM) {
            request.action = MOTOR_ACTION_SET_RPM;
        } else if (frame->type == HOST_MSG_SET_CURRENT) {
            request.action = MOTOR_ACTION_SET_CURRENT;
        } else {
            request.action = MOTOR_ACTION_SET_POSITION;
            if ((uint16_t)request.target_value > M0601_POSITION_RAW_MAX) {
                status = HOST_STATUS_RANGE;
                break;
            }
        }
        const bool active = frame->type == HOST_MSG_SET_POSITION ||
                            request.target_value != 0;
        response = execute_owned_control(source, &request, active, &status);
        break;
    }
    case HOST_MSG_JOYSTICK: {
        if (frame->payload_length != 8u) {
            status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.id = frame->payload[0];
        if (!valid_motor_id(request.id)) {
            status = HOST_STATUS_RANGE;
            break;
        }
        if (frame->payload[7] == 0u) {
            request.action = MOTOR_ACTION_STOP;
            request.brake = M0601_BRAKE_OFF;
        } else {
            request.action = MOTOR_ACTION_SET_RPM;
            request.target_value = joystick_single_wheel_rpm(
                robot_read_i16_le(&frame->payload[3]),
                robot_read_u16_le(&frame->payload[5]));
            request.accel = M0601_ACCEL_DEFAULT;
            request.brake = M0601_BRAKE_OFF;
        }
        response = execute_owned_control(source, &request,
                                         request.target_value != 0, &status);
        break;
    }
    case HOST_MSG_SET_DUAL_RPM:
        if (frame->payload_length != 8u) {
            status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_SET_DUAL_RPM;
        request.id = frame->payload[0];
        request.right_id = frame->payload[1];
        request.target_value = robot_read_i16_le(&frame->payload[2]);
        request.right_target_value = robot_read_i16_le(&frame->payload[4]);
        request.accel = frame->payload[6];
        request.brake = frame->payload[7];
        response = execute_owned_control(
            source, &request,
            request.target_value != 0 || request.right_target_value != 0,
            &status);
        break;
    case HOST_MSG_STOP:
        if (frame->payload_length != 2u) {
            status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_STOP;
        request.id = frame->payload[0];
        request.brake = frame->payload[1];
        if (!valid_motor_id(request.id)) {
            status = HOST_STATUS_RANGE;
            break;
        }
        response = execute_control(&request);
        status = map_motor_status(response.status);
        if (response.status == M0601_OK) release_control(HOST_SOURCE_NONE);
        break;
    case HOST_MSG_STOP_DUAL:
        if (frame->payload_length != 3u) {
            status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_STOP_DUAL;
        request.id = frame->payload[0];
        request.right_id = frame->payload[1];
        request.brake = frame->payload[2];
        response = execute_control(&request);
        status = map_motor_status(response.status);
        release_control(HOST_SOURCE_NONE);
        break;
    case HOST_MSG_CONTROL_KEEPALIVE:
        if (frame->payload_length != 1u) status = HOST_STATUS_BAD_LENGTH;
        else if (control_owner() != source) status = HOST_STATUS_BUSY;
        else status = map_motor_status(motor_service_keepalive(frame->payload[0]));
        break;
    case HOST_MSG_DUAL_KEEPALIVE:
        if (frame->payload_length != 2u) status = HOST_STATUS_BAD_LENGTH;
        else if (control_owner() != source) status = HOST_STATUS_BUSY;
        else status = map_motor_status(motor_service_dual_keepalive(
            frame->payload[0], frame->payload[1]));
        break;
    case HOST_MSG_QUERY_MOTOR:
        if (frame->payload_length != 1u) {
            status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_QUERY;
        request.id = frame->payload[0];
        if (!valid_motor_id(request.id)) {
            status = HOST_STATUS_RANGE;
            break;
        }
        response = motor_service_execute(&request);
        status = map_motor_status(response.status);
        break;
    case HOST_MSG_QUERY_UNIQUE_ID:
        if (frame->payload_length != 0u) {
            status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_QUERY_UNIQUE_ID;
        response = motor_service_execute(&request);
        status = map_motor_status(response.status);
        break;
    case HOST_MSG_SET_ID:
        if (frame->payload_length != 4u) {
            status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        if (robot_read_u16_le(&frame->payload[2]) != HOST_SET_ID_CONFIRM) {
            status = HOST_STATUS_PRECONDITION;
            break;
        }
        request.action = MOTOR_ACTION_SET_ID;
        request.expected_old_id = frame->payload[0];
        request.new_id = frame->payload[1];
        if (!valid_motor_id(request.expected_old_id) ||
            !valid_motor_id(request.new_id)) {
            status = HOST_STATUS_RANGE;
            break;
        }
        response = motor_service_execute(&request);
        status = map_motor_status(response.status);
        break;
    case HOST_MSG_SET_MODE:
        if (frame->payload_length != 2u) {
            status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_SET_MODE;
        request.id = frame->payload[0];
        request.mode = frame->payload[1];
        if (!valid_motor_id(request.id) ||
            (request.mode != M0601_MODE_CURRENT &&
             request.mode != M0601_MODE_SPEED &&
             request.mode != M0601_MODE_POSITION)) {
            status = HOST_STATUS_RANGE;
            break;
        }
        response = motor_service_execute(&request);
        status = map_motor_status(response.status);
        break;
    default:
        status = HOST_STATUS_UNSUPPORTED;
        break;
    }
    if ((frame->flags & HOST_FLAG_ACK_REQUIRED) != 0u ||
        status != HOST_STATUS_OK) {
        send_ack(source, frame, status, response.detail);
    }
}

static void host_uart_receive_task(void *argument)
{
    (void)argument;
    uint8_t received[128];
    host_stream_parser_init(&s_uart_context.parser);
    for (;;) {
        size_t length = 0u;
        if (esp32_host_uart_read(received, sizeof(received), 20, &length) == ESP_OK &&
            length > 0u) {
            host_stream_parser_feed(&s_uart_context.parser, received, length,
                                    dispatch_host_frame, &s_uart_context);
        }
    }
}

static void wifi_receive(void *context, const uint8_t *data, size_t length)
{
    host_source_context_t *source = (host_source_context_t *)context;
    host_stream_parser_feed(&source->parser, data, length,
                            dispatch_host_frame, source);
}

static void stop_owned_motion(host_source_t source)
{
    if (control_owner() != source) return;
    motor_chassis_snapshot_t snapshot;
    motor_service_get_chassis_snapshot(&snapshot);
    motor_request_t stop = {.brake = M0601_BRAKE_ON};
    if (snapshot.dual_control_active) {
        stop.action = MOTOR_ACTION_STOP_DUAL;
        stop.id = CONFIG_ROBOT_LEFT_MOTOR_ID;
        stop.right_id = CONFIG_ROBOT_RIGHT_MOTOR_ID;
    } else {
        stop.action = MOTOR_ACTION_STOP;
        stop.id = snapshot.selected.motor_id;
    }
    (void)motor_service_execute(&stop);
    release_control(source);
}

static void wifi_disconnected(void *context)
{
    host_source_context_t *source = (host_source_context_t *)context;
    stop_owned_motion(source->source);
    host_stream_parser_init(&source->parser);
}

static void pack_legacy_heartbeat(uint8_t payload[HOST_HEARTBEAT_SIZE],
                                  const motor_snapshot_t *snapshot)
{
    memset(payload, 0, HOST_HEARTBEAT_SIZE);
    robot_write_u32_le(&payload[0], snapshot->uptime_ms);
    payload[4] = snapshot->motor_id;
    payload[5] = snapshot->mode;
    payload[6] = snapshot->state;
    payload[7] = snapshot->fault;
    robot_write_i16_le(&payload[8], snapshot->target_value);
    robot_write_i16_le(&payload[10], snapshot->actual_rpm);
    robot_write_i16_le(&payload[12], snapshot->current_raw);
    robot_write_u16_le(&payload[14], snapshot->drive_position_raw);
    payload[16] = snapshot->query_position_u8;
    uint32_t age = snapshot->uptime_ms - snapshot->last_feedback_ms;
    if (age > UINT16_MAX) age = UINT16_MAX;
    robot_write_u16_le(&payload[18], (uint16_t)age);
    robot_write_u32_le(&payload[20], snapshot->valid_feedback_count);
    robot_write_u16_le(&payload[24], snapshot->crc_error_count);
    robot_write_u16_le(&payload[26], snapshot->timeout_count);
    robot_write_u16_le(&payload[28], snapshot->watchdog_stop_count);
}

static void pack_motor_record(uint8_t *payload,
                              const motor_wheel_snapshot_t *wheel,
                              uint32_t uptime_ms)
{
    payload[0] = wheel->motor_id;
    payload[1] = wheel->mode;
    payload[2] = wheel->state;
    payload[3] = wheel->fault;
    robot_write_i16_le(&payload[4], wheel->target_value);
    robot_write_i16_le(&payload[6], wheel->actual_rpm);
    robot_write_i16_le(&payload[8], wheel->current_raw);
    robot_write_u16_le(&payload[10], wheel->drive_position_raw);
    payload[12] = wheel->query_position_u8;
    payload[13] = 0u;
    uint32_t age = uptime_ms - wheel->last_feedback_ms;
    if (age > UINT16_MAX) age = UINT16_MAX;
    robot_write_u16_le(&payload[14], (uint16_t)age);
    robot_write_u32_le(&payload[16], wheel->valid_feedback_count);
    robot_write_u16_le(&payload[20], wheel->crc_error_count);
    robot_write_u16_le(&payload[22], wheel->timeout_count);
}

static void pack_chassis_telemetry(uint8_t payload[HOST_CHASSIS_TELEMETRY_SIZE],
                                   const motor_chassis_snapshot_t *snapshot)
{
    memset(payload, 0, HOST_CHASSIS_TELEMETRY_SIZE);
    robot_write_u32_le(&payload[0], snapshot->uptime_ms);
    payload[4] = (uint8_t)control_owner();
    uint8_t flags = snapshot->control_active ? HOST_CHASSIS_FLAG_CONTROL_ACTIVE : 0u;
    if (snapshot->wheel[MOTOR_LEFT_INDEX].state == MOTOR_STATE_ESTOP ||
        snapshot->wheel[MOTOR_RIGHT_INDEX].state == MOTOR_STATE_ESTOP) {
        flags |= HOST_CHASSIS_FLAG_ESTOP;
    }
    if (esp32_wifi_tcp_has_ip()) flags |= HOST_CHASSIS_FLAG_WIFI_UP;
    const uint32_t now = snapshot->uptime_ms;
    if (snapshot->wheel[MOTOR_LEFT_INDEX].address_confirmed &&
        now - snapshot->wheel[MOTOR_LEFT_INDEX].last_feedback_ms <=
            ROBOT_MOTOR_FEEDBACK_FRESH_MS) flags |= HOST_CHASSIS_FLAG_LEFT_VALID;
    if (snapshot->wheel[MOTOR_RIGHT_INDEX].address_confirmed &&
        now - snapshot->wheel[MOTOR_RIGHT_INDEX].last_feedback_ms <=
            ROBOT_MOTOR_FEEDBACK_FRESH_MS) flags |= HOST_CHASSIS_FLAG_RIGHT_VALID;
    payload[5] = flags;
    robot_write_u16_le(&payload[6], snapshot->watchdog_stop_count);
    pack_motor_record(&payload[8], &snapshot->wheel[MOTOR_LEFT_INDEX], now);
    pack_motor_record(&payload[32], &snapshot->wheel[MOTOR_RIGHT_INDEX], now);
}

static void motor_telemetry_task(void *argument)
{
    (void)argument;
    for (;;) {
        motor_snapshot_t legacy;
        motor_chassis_snapshot_t chassis;
        uint8_t legacy_payload[HOST_HEARTBEAT_SIZE];
        uint8_t chassis_payload[HOST_CHASSIS_TELEMETRY_SIZE];
        motor_service_get_snapshot(&legacy);
        motor_service_get_chassis_snapshot(&chassis);
        if (!chassis.control_active) release_control(HOST_SOURCE_NONE);
        pack_legacy_heartbeat(legacy_payload, &legacy);
        pack_chassis_telemetry(chassis_payload, &chassis);
        broadcast_frame(HOST_MSG_HEARTBEAT, s_legacy_sequence++, legacy_payload,
                        sizeof(legacy_payload));
        broadcast_frame(HOST_MSG_CHASSIS_TELEMETRY, s_chassis_sequence++,
                        chassis_payload, sizeof(chassis_payload));
        vTaskDelay(pdMS_TO_TICKS(CONFIG_ROBOT_TELEMETRY_PERIOD_MS));
    }
}

static void imu_telemetry_task(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        bmi088_snapshot_t snapshot;
        uint8_t payload[HOST_IMU_TELEMETRY_SIZE] = {0};
        bmi088_service_get_snapshot(&snapshot);
        robot_write_u64_le(&payload[0], snapshot.timestamp_us);
        if (snapshot.online) payload[8] |= HOST_IMU_FLAG_ONLINE;
        if (snapshot.calibrated) payload[8] |= HOST_IMU_FLAG_CALIBRATED;
        if (snapshot.sample_valid) payload[8] |= HOST_IMU_FLAG_SAMPLE_VALID;
        for (size_t i = 0; i < 3; ++i) {
            robot_write_f32_le(&payload[12 + i * 4u], snapshot.accel_mps2[i]);
            robot_write_f32_le(&payload[24 + i * 4u], snapshot.gyro_rads[i]);
        }
        robot_write_u32_le(&payload[36], snapshot.sample_count);
        robot_write_u16_le(&payload[40], snapshot.read_error_count);
        robot_write_u16_le(&payload[42], snapshot.init_error_count);
        broadcast_frame(HOST_MSG_IMU_TELEMETRY, s_imu_sequence++, payload,
                        sizeof(payload));
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ROBOT_IMU_SAMPLE_PERIOD_MS));
    }
}

esp_err_t host_link_service_start(void)
{
    s_uart_write_mutex = xSemaphoreCreateMutex();
    s_owner_mutex = xSemaphoreCreateMutex();
    if (s_uart_write_mutex == NULL || s_owner_mutex == NULL) return ESP_ERR_NO_MEM;
    const esp_err_t uart_status = esp32_host_uart_init();
    if (uart_status != ESP_OK) return uart_status;
    host_stream_parser_init(&s_tcp_context.parser);
    (void)esp32_wifi_tcp_start(wifi_receive, wifi_disconnected, &s_tcp_context);
    if (xTaskCreate(host_uart_receive_task, "host_uart_rx", 4096, NULL, 10, NULL) !=
        pdPASS) return ESP_ERR_NO_MEM;
    if (xTaskCreate(motor_telemetry_task, "motor_telemetry", 4096, NULL, 8, NULL) !=
        pdPASS) return ESP_ERR_NO_MEM;
    if (xTaskCreate(imu_telemetry_task, "imu_telemetry", 3072, NULL, 8, NULL) !=
        pdPASS) return ESP_ERR_NO_MEM;
    return ESP_OK;
}
