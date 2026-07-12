#include "services/host_link_service.h"

#include <string.h>

#include "algorithms/joystick_mapping.h"
#include "common/byte_order.h"
#include "config/robot_config.h"
#include "esp32_drivers/host_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "protocols/host_frame.h"
#include "protocols/host_messages.h"
#include "protocols/m0601c111_motor.h"
#include "services/motor_service.h"

static SemaphoreHandle_t s_host_write_mutex;
static uint8_t s_telemetry_sequence;

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

static void send_frame(uint8_t type,
                       uint8_t sequence,
                       const uint8_t *payload,
                       uint16_t payload_length)
{
    uint8_t encoded[HOST_FRAME_MAX_SIZE];
    const size_t length = host_frame_encode(type, sequence, 0u,
                                             payload, payload_length,
                                             encoded, sizeof(encoded));
    if (length == 0u) return;
    xSemaphoreTake(s_host_write_mutex, portMAX_DELAY);
    (void)esp32_host_uart_write(encoded, length, 100);
    xSemaphoreGive(s_host_write_mutex);
}

static void send_ack(const host_frame_t *request,
                     host_status_t status,
                     int16_t detail)
{
    uint8_t payload[4];
    payload[0] = request->type;
    payload[1] = (uint8_t)status;
    robot_write_i16_le(&payload[2], detail);
    send_frame(HOST_MSG_ACK, request->sequence, payload, sizeof(payload));
}

static bool valid_motor_id(uint8_t id)
{
    return id != M0601_ID_QUERY_ADDRESS;
}

static motor_response_t execute_and_mark(const motor_request_t *request)
{
    motor_service_mark_control_received();
    return motor_service_execute(request);
}

static void dispatch_host_frame(void *context, const host_frame_t *frame)
{
    (void)context;
    motor_request_t request = {0};
    motor_response_t response = {.status = M0601_OK, .detail = 0};
    host_status_t host_status = HOST_STATUS_OK;

    switch (frame->type) {
    case HOST_MSG_HELLO:
        if (frame->payload_length != 0u) host_status = HOST_STATUS_BAD_LENGTH;
        break;

    case HOST_MSG_SET_SINGLE_RPM:
        if (frame->payload_length != 5u) {
            host_status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_SET_RPM;
        request.id = frame->payload[0];
        request.target_rpm = robot_read_i16_le(&frame->payload[1]);
        request.accel = frame->payload[3];
        request.brake = frame->payload[4];
        if (!valid_motor_id(request.id)) {
            host_status = HOST_STATUS_RANGE;
            break;
        }
        response = execute_and_mark(&request);
        host_status = map_motor_status(response.status);
        break;

    case HOST_MSG_JOYSTICK:
        if (frame->payload_length != 8u) {
            host_status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.id = frame->payload[0];
        if (!valid_motor_id(request.id)) {
            host_status = HOST_STATUS_RANGE;
            break;
        }
        if (frame->payload[7] == 0u) {
            request.action = MOTOR_ACTION_STOP;
            request.brake = M0601_BRAKE_OFF;
        } else {
            request.action = MOTOR_ACTION_SET_RPM;
            request.target_rpm = joystick_single_wheel_rpm(
                robot_read_i16_le(&frame->payload[3]),
                robot_read_u16_le(&frame->payload[5]));
            request.accel = M0601_ACCEL_DEFAULT;
            request.brake = M0601_BRAKE_OFF;
        }
        response = execute_and_mark(&request);
        host_status = map_motor_status(response.status);
        break;

    case HOST_MSG_STOP:
        if (frame->payload_length != 2u) {
            host_status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_STOP;
        request.id = frame->payload[0];
        request.brake = frame->payload[1];
        if (!valid_motor_id(request.id)) {
            host_status = HOST_STATUS_RANGE;
            break;
        }
        response = execute_and_mark(&request);
        host_status = map_motor_status(response.status);
        break;

    case HOST_MSG_QUERY_MOTOR:
        if (frame->payload_length != 1u) {
            host_status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_QUERY;
        request.id = frame->payload[0];
        response = motor_service_execute(&request);
        host_status = map_motor_status(response.status);
        break;

    case HOST_MSG_QUERY_UNIQUE_ID:
        if (frame->payload_length != 0u) {
            host_status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_QUERY_UNIQUE_ID;
        response = motor_service_execute(&request);
        host_status = map_motor_status(response.status);
        break;

    case HOST_MSG_SET_ID:
        if (frame->payload_length != 4u) {
            host_status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        if (robot_read_u16_le(&frame->payload[2]) != HOST_SET_ID_CONFIRM) {
            host_status = HOST_STATUS_PRECONDITION;
            break;
        }
        request.action = MOTOR_ACTION_SET_ID;
        request.expected_old_id = frame->payload[0];
        request.new_id = frame->payload[1];
        if (!valid_motor_id(request.expected_old_id) ||
            !valid_motor_id(request.new_id)) {
            host_status = HOST_STATUS_RANGE;
            break;
        }
        response = motor_service_execute(&request);
        host_status = map_motor_status(response.status);
        break;

    case HOST_MSG_SET_MODE:
        if (frame->payload_length != 2u) {
            host_status = HOST_STATUS_BAD_LENGTH;
            break;
        }
        request.action = MOTOR_ACTION_SET_MODE;
        request.id = frame->payload[0];
        request.mode = frame->payload[1];
        response = motor_service_execute(&request);
        host_status = map_motor_status(response.status);
        break;

    default:
        host_status = HOST_STATUS_UNSUPPORTED;
        break;
    }

    send_ack(frame, host_status, response.detail);
}

static void host_uart_receive_task(void *argument)
{
    (void)argument;
    host_stream_parser_t parser;
    uint8_t received[128];
    host_stream_parser_init(&parser);

    for (;;) {
        size_t length = 0u;
        const esp_err_t status = esp32_host_uart_read(received,
                                                      sizeof(received),
                                                      20,
                                                      &length);
        if (status == ESP_OK && length > 0u) {
            host_stream_parser_feed(&parser, received, length,
                                    dispatch_host_frame, NULL);
        }
    }
}

static void telemetry_task(void *argument)
{
    (void)argument;
    for (;;) {
        motor_snapshot_t snapshot;
        uint8_t payload[HOST_HEARTBEAT_SIZE] = {0};
        motor_service_get_snapshot(&snapshot);

        robot_write_u32_le(&payload[0], snapshot.uptime_ms);
        payload[4] = snapshot.motor_id;
        payload[5] = snapshot.mode;
        payload[6] = snapshot.state;
        payload[7] = snapshot.fault;
        robot_write_i16_le(&payload[8], snapshot.target_rpm);
        robot_write_i16_le(&payload[10], snapshot.actual_rpm);
        robot_write_i16_le(&payload[12], snapshot.current_raw);
        robot_write_u16_le(&payload[14], snapshot.drive_position_raw);
        payload[16] = snapshot.query_position_u8;
        payload[17] = snapshot.temperature_raw;
        uint32_t age = snapshot.uptime_ms - snapshot.last_feedback_ms;
        if (age > UINT16_MAX) age = UINT16_MAX;
        robot_write_u16_le(&payload[18], (uint16_t)age);
        robot_write_u32_le(&payload[20], snapshot.valid_feedback_count);
        robot_write_u16_le(&payload[24], snapshot.crc_error_count);
        robot_write_u16_le(&payload[26], snapshot.timeout_count);
        robot_write_u16_le(&payload[28], snapshot.watchdog_stop_count);

        send_frame(HOST_MSG_HEARTBEAT, s_telemetry_sequence++, payload, sizeof(payload));
        vTaskDelay(pdMS_TO_TICKS(CONFIG_ROBOT_TELEMETRY_PERIOD_MS));
    }
}

esp_err_t host_link_service_start(void)
{
    s_host_write_mutex = xSemaphoreCreateMutex();
    if (s_host_write_mutex == NULL) return ESP_ERR_NO_MEM;
    const esp_err_t uart_status = esp32_host_uart_init();
    if (uart_status != ESP_OK) return uart_status;

    if (xTaskCreate(host_uart_receive_task, "host_uart_rx", 4096, NULL, 10, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(telemetry_task, "telemetry", 3072, NULL, 8, NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
