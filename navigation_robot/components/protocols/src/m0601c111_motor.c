#include "protocols/m0601c111_motor.h"

#include <string.h>

#define M0601_CRC_POLY_REFLECTED 0x8Cu
#define M0601_REPEAT_DELAY_MS 1u

static bool valid_id(uint8_t id) { return id != M0601_ID_QUERY_ADDRESS; }
static bool valid_mode(uint8_t mode)
{
    return mode == M0601_MODE_CURRENT || mode == M0601_MODE_SPEED ||
           mode == M0601_MODE_POSITION;
}
static void write_u16_be(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value >> 8);
    dst[1] = (uint8_t)value;
}
static uint16_t read_u16_be(const uint8_t *src)
{
    return ((uint16_t)src[0] << 8) | src[1];
}
static int16_t read_i16_be(const uint8_t *src) { return (int16_t)read_u16_be(src); }
static void finish_crc(uint8_t *frame)
{
    frame[9] = m0601_crc8_maxim(frame, 9u);
}

m0601_status_t m0601_init(m0601_motor_t *motor,
                          const m0601_transport_t *transport,
                          uint8_t default_id,
                          uint32_t timeout_ms)
{
    if (motor == NULL || transport == NULL || transport->send == NULL ||
        !valid_id(default_id)) {
        return motor == NULL || transport == NULL || transport->send == NULL
                   ? M0601_ERROR_NULL
                   : M0601_ERROR_RANGE;
    }
    motor->transport = *transport;
    motor->default_id = default_id;
    motor->timeout_ms = timeout_ms;
    return M0601_OK;
}

uint8_t m0601_crc8_maxim(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    if (data == NULL) {
        return 0;
    }
    for (size_t index = 0; index < len; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 1u) != 0u
                      ? (uint8_t)((crc >> 1) ^ M0601_CRC_POLY_REFLECTED)
                      : (uint8_t)(crc >> 1);
        }
    }
    return crc;
}

bool m0601_check_crc(const uint8_t frame[M0601_FRAME_SIZE])
{
    return frame != NULL && frame[9] == m0601_crc8_maxim(frame, 9u);
}

static m0601_status_t build_drive(uint8_t id,
                                  uint16_t target,
                                  uint8_t accel,
                                  uint8_t brake,
                                  uint8_t *frame)
{
    if (frame == NULL) return M0601_ERROR_NULL;
    if (!valid_id(id)) return M0601_ERROR_RANGE;
    (void)memset(frame, 0, M0601_FRAME_SIZE);
    frame[0] = id;
    frame[1] = M0601_CMD_DRIVE;
    write_u16_be(&frame[2], target);
    frame[6] = accel;
    frame[7] = brake;
    finish_crc(frame);
    return M0601_OK;
}

m0601_status_t m0601_build_drive_raw(uint8_t id, int16_t target, uint8_t accel, uint8_t brake, uint8_t *frame)
{
    if (frame == NULL) return M0601_ERROR_NULL;
    if (target < M0601_CURRENT_RAW_MIN) return M0601_ERROR_RANGE;
    return build_drive(id, (uint16_t)target, accel, brake, frame);
}
m0601_status_t m0601_build_drive_current(uint8_t id, int16_t target, uint8_t accel, uint8_t brake, uint8_t *frame)
{
    return m0601_build_drive_raw(id, target, accel, brake, frame);
}
m0601_status_t m0601_build_drive_speed(uint8_t id, int16_t rpm, uint8_t accel, uint8_t brake, uint8_t *frame)
{
    if (frame == NULL) return M0601_ERROR_NULL;
    if (rpm < M0601_SPEED_RPM_MIN || rpm > M0601_SPEED_RPM_MAX) return M0601_ERROR_RANGE;
    return build_drive(id, (uint16_t)rpm, accel, brake, frame);
}
m0601_status_t m0601_build_drive_position(uint8_t id, uint16_t target, uint8_t accel, uint8_t brake, uint8_t *frame)
{
    if (frame == NULL) return M0601_ERROR_NULL;
    if (target > M0601_POSITION_RAW_MAX) return M0601_ERROR_RANGE;
    return build_drive(id, target, accel, brake, frame);
}
m0601_status_t m0601_build_query(uint8_t id, uint8_t *frame)
{
    if (frame == NULL) return M0601_ERROR_NULL;
    if (!valid_id(id)) return M0601_ERROR_RANGE;
    (void)memset(frame, 0, M0601_FRAME_SIZE);
    frame[0] = id;
    frame[1] = M0601_CMD_QUERY;
    finish_crc(frame);
    return M0601_OK;
}
m0601_status_t m0601_build_id_query(uint8_t *frame)
{
    if (frame == NULL) return M0601_ERROR_NULL;
    (void)memset(frame, 0, M0601_FRAME_SIZE);
    frame[0] = M0601_ID_QUERY_ADDRESS;
    frame[1] = M0601_CMD_DRIVE;
    finish_crc(frame);
    return M0601_OK;
}
m0601_status_t m0601_build_set_mode(uint8_t id, uint8_t mode, uint8_t *frame)
{
    if (frame == NULL) return M0601_ERROR_NULL;
    if (!valid_id(id) || !valid_mode(mode)) return M0601_ERROR_RANGE;
    (void)memset(frame, 0, M0601_FRAME_SIZE);
    frame[0] = id;
    frame[1] = M0601_CMD_SET_MODE;
    frame[9] = mode;
    return M0601_OK;
}
m0601_status_t m0601_build_set_id(uint8_t new_id, uint8_t *frame)
{
    if (frame == NULL) return M0601_ERROR_NULL;
    if (!valid_id(new_id)) return M0601_ERROR_RANGE;
    (void)memset(frame, 0, M0601_FRAME_SIZE);
    frame[0] = M0601_SET_ID_HEAD0;
    frame[1] = M0601_SET_ID_HEAD1;
    frame[2] = M0601_SET_ID_HEAD2;
    frame[3] = new_id;
    return M0601_OK;
}

static m0601_status_t parse_common(const uint8_t *frame,
                                   uint8_t *id,
                                   uint8_t *mode,
                                   int16_t *current,
                                   int16_t *speed,
                                   uint8_t *fault)
{
    if (frame == NULL || id == NULL || mode == NULL || current == NULL ||
        speed == NULL || fault == NULL) return M0601_ERROR_NULL;
    if (!m0601_check_crc(frame)) return M0601_ERROR_CRC;
    if (!valid_mode(frame[1])) return M0601_ERROR_FRAME;
    *id = frame[0];
    *mode = frame[1];
    *current = read_i16_be(&frame[2]);
    *speed = read_i16_be(&frame[4]);
    *fault = frame[8];
    return M0601_OK;
}
m0601_status_t m0601_parse_drive_feedback(const uint8_t *frame, m0601_drive_feedback_t *feedback)
{
    if (feedback == NULL) return M0601_ERROR_NULL;
    m0601_status_t status = parse_common(frame, &feedback->id, &feedback->mode,
                                          &feedback->torque_current_raw,
                                          &feedback->speed_rpm, &feedback->fault);
    if (status == M0601_OK) feedback->position_raw = read_u16_be(&frame[6]);
    return status;
}
m0601_status_t m0601_parse_query_feedback(const uint8_t *frame, m0601_query_feedback_t *feedback)
{
    if (feedback == NULL) return M0601_ERROR_NULL;
    m0601_status_t status = parse_common(frame, &feedback->id, &feedback->mode,
                                          &feedback->torque_current_raw,
                                          &feedback->speed_rpm, &feedback->fault);
    if (status == M0601_OK) {
        feedback->temperature_raw = frame[6];
        feedback->position_u8 = frame[7];
    }
    return status;
}

static m0601_status_t send_frame(m0601_motor_t *motor, const uint8_t *frame)
{
    if (motor == NULL || motor->transport.send == NULL || frame == NULL) return M0601_ERROR_NULL;
    return motor->transport.send(motor->transport.ctx, frame, M0601_FRAME_SIZE, motor->timeout_ms);
}
static m0601_status_t transact(m0601_motor_t *motor, const uint8_t *request, uint8_t *response)
{
    m0601_status_t status = send_frame(motor, request);
    if (status != M0601_OK) return status;
    if (motor->transport.recv == NULL) return M0601_ERROR_NULL;
    status = motor->transport.recv(motor->transport.ctx, response, M0601_FRAME_SIZE, motor->timeout_ms);
    if (status != M0601_OK) return status;
    return m0601_check_crc(response) ? M0601_OK : M0601_ERROR_CRC;
}
static m0601_status_t drive_request(m0601_motor_t *motor, const uint8_t *request, bool id_query, m0601_drive_feedback_t *feedback)
{
    uint8_t response[M0601_FRAME_SIZE];
    m0601_drive_feedback_t parsed;
    m0601_status_t status = transact(motor, request, response);
    if (status != M0601_OK) return status;
    status = m0601_parse_drive_feedback(response, &parsed);
    if (status != M0601_OK) return status;
    if ((!id_query && parsed.id != request[0]) || (id_query && !valid_id(parsed.id))) return M0601_ERROR_FRAME;
    if (feedback != NULL) *feedback = parsed;
    return M0601_OK;
}
m0601_status_t m0601_drive_raw(m0601_motor_t *motor, uint8_t id, int16_t target, uint8_t accel, uint8_t brake, m0601_drive_feedback_t *feedback)
{
    uint8_t request[M0601_FRAME_SIZE];
    m0601_status_t status = m0601_build_drive_raw(id, target, accel, brake, request);
    return status == M0601_OK ? drive_request(motor, request, false, feedback) : status;
}
m0601_status_t m0601_drive_default(m0601_motor_t *motor, int16_t target, uint8_t accel, uint8_t brake, m0601_drive_feedback_t *feedback)
{
    return motor == NULL ? M0601_ERROR_NULL : m0601_drive_raw(motor, motor->default_id, target, accel, brake, feedback);
}
m0601_status_t m0601_drive_current(m0601_motor_t *motor, uint8_t id, int16_t target, uint8_t accel, uint8_t brake, m0601_drive_feedback_t *feedback)
{
    return m0601_drive_raw(motor, id, target, accel, brake, feedback);
}
m0601_status_t m0601_drive_speed(m0601_motor_t *motor, uint8_t id, int16_t rpm, uint8_t accel, uint8_t brake, m0601_drive_feedback_t *feedback)
{
    uint8_t request[M0601_FRAME_SIZE];
    m0601_status_t status = m0601_build_drive_speed(id, rpm, accel, brake, request);
    return status == M0601_OK ? drive_request(motor, request, false, feedback) : status;
}
m0601_status_t m0601_drive_position(m0601_motor_t *motor, uint8_t id, uint16_t target, uint8_t accel, uint8_t brake, m0601_drive_feedback_t *feedback)
{
    uint8_t request[M0601_FRAME_SIZE];
    m0601_status_t status = m0601_build_drive_position(id, target, accel, brake, request);
    return status == M0601_OK ? drive_request(motor, request, false, feedback) : status;
}
m0601_status_t m0601_query(m0601_motor_t *motor, uint8_t id, m0601_query_feedback_t *feedback)
{
    uint8_t request[M0601_FRAME_SIZE], response[M0601_FRAME_SIZE];
    m0601_query_feedback_t parsed;
    m0601_status_t status = m0601_build_query(id, request);
    if (status != M0601_OK) return status;
    status = transact(motor, request, response);
    if (status != M0601_OK) return status;
    status = m0601_parse_query_feedback(response, &parsed);
    if (status != M0601_OK) return status;
    if (parsed.id != id) return M0601_ERROR_FRAME;
    if (feedback != NULL) *feedback = parsed;
    return M0601_OK;
}
m0601_status_t m0601_query_default(m0601_motor_t *motor, m0601_query_feedback_t *feedback)
{
    return motor == NULL ? M0601_ERROR_NULL : m0601_query(motor, motor->default_id, feedback);
}
m0601_status_t m0601_query_id(m0601_motor_t *motor, m0601_drive_feedback_t *feedback)
{
    uint8_t request[M0601_FRAME_SIZE];
    m0601_status_t status = m0601_build_id_query(request);
    return status == M0601_OK ? drive_request(motor, request, true, feedback) : status;
}
m0601_status_t m0601_set_mode(m0601_motor_t *motor, uint8_t id, uint8_t mode)
{
    uint8_t request[M0601_FRAME_SIZE];
    m0601_status_t status = m0601_build_set_mode(id, mode, request);
    return status == M0601_OK ? send_frame(motor, request) : status;
}
m0601_status_t m0601_set_id(m0601_motor_t *motor, uint8_t new_id)
{
    uint8_t request[M0601_FRAME_SIZE];
    m0601_status_t status = m0601_build_set_id(new_id, request);
    if (status != M0601_OK) return status;
    for (uint8_t index = 0; index < M0601_SET_ID_REPEAT_COUNT; ++index) {
        status = send_frame(motor, request);
        if (status != M0601_OK) return status;
        if (index + 1u < M0601_SET_ID_REPEAT_COUNT && motor->transport.delay_ms != NULL) {
            motor->transport.delay_ms(motor->transport.ctx, M0601_REPEAT_DELAY_MS);
        }
    }
    return M0601_OK;
}
