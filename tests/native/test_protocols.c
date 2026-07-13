#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "protocols/host_frame.h"
#include "protocols/host_messages.h"
#include "protocols/m0601c111_motor.h"

static unsigned callback_count;
static host_frame_t last_frame;

static void capture_frame(void *context, const host_frame_t *frame)
{
    (void)context;
    ++callback_count;
    last_frame = *frame;
}

typedef struct {
    unsigned sends;
    unsigned delays;
    uint8_t last_frame[M0601_FRAME_SIZE];
} fake_transport_t;

static m0601_status_t fake_send(void *context,
                                const uint8_t *data,
                                size_t length,
                                uint32_t timeout_ms)
{
    fake_transport_t *fake = context;
    (void)timeout_ms;
    assert(length == M0601_FRAME_SIZE);
    ++fake->sends;
    memcpy(fake->last_frame, data, length);
    return M0601_OK;
}

static void fake_delay(void *context, uint32_t delay_ms)
{
    fake_transport_t *fake = context;
    assert(delay_ms == 1u);
    ++fake->delays;
}

static void test_host_protocol(void)
{
    static const uint8_t expected_ping[] = {
        0xAA, 0x55, 0x01, 0x01, 0x2A, 0x00, 0x00, 0x00, 0x04, 0xBE,
    };
    uint8_t encoded[HOST_FRAME_MAX_SIZE];
    const size_t length = host_frame_encode(0x01, 0x2A, 0, NULL, 0,
                                             encoded, sizeof(encoded));
    assert(length == sizeof(expected_ping));
    assert(memcmp(encoded, expected_ping, sizeof(expected_ping)) == 0);

    host_stream_parser_t parser;
    host_stream_parser_init(&parser);
    callback_count = 0;
    for (size_t index = 0; index < length; ++index) {
        host_stream_parser_feed(&parser, &encoded[index], 1, capture_frame, NULL);
    }
    assert(callback_count == 1);
    assert(last_frame.type == 0x01 && last_frame.sequence == 0x2A);

    encoded[length - 1] ^= 0xFF;
    host_stream_parser_feed(&parser, encoded, length, capture_frame, NULL);
    assert(parser.crc_errors == 1);

    static const uint8_t expected_dual[] = {
        0xAA, 0x55, 0x01, 0x1A, 0x01, 0x01, 0x08, 0x00,
        0x01, 0x02, 0x19, 0x00, 0x19, 0x00, 0x00, 0x00, 0xD2, 0x49,
    };
    const uint8_t dual_payload[] = {1, 2, 25, 0, 25, 0, 0, 0};
    const size_t dual_length = host_frame_encode(
        HOST_MSG_SET_DUAL_RPM, 1, HOST_FLAG_ACK_REQUIRED,
        dual_payload, sizeof(dual_payload), encoded, sizeof(encoded));
    assert(dual_length == sizeof(expected_dual));
    assert(memcmp(encoded, expected_dual, sizeof(expected_dual)) == 0);
    assert(HOST_CHASSIS_TELEMETRY_SIZE == 56u);
    assert(HOST_IMU_TELEMETRY_SIZE == 44u);
}

static void test_m0601_protocol(void)
{
    static const uint8_t expected_speed[] = {
        0x01, 0x64, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18,
    };
    uint8_t frame[M0601_FRAME_SIZE];
    assert(m0601_build_drive_speed(1, 30, 0, 0, frame) == M0601_OK);
    assert(memcmp(frame, expected_speed, sizeof(frame)) == 0);
    assert(m0601_build_drive_speed(1, 331, 0, 0, frame) == M0601_ERROR_RANGE);
    assert(m0601_build_drive_raw(1, INT16_MIN, 0, 0, frame) == M0601_ERROR_RANGE);

    static const uint8_t expected_positive_speed[] = {
        0x01, 0x64, 0x00, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x53,
    };
    static const uint8_t expected_zero[] = {
        0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50,
    };
    static const uint8_t expected_negative_speed[] = {
        0x01, 0x64, 0xFF, 0x6A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5A,
    };
    static const uint8_t expected_position_quarter[] = {
        0x01, 0x64, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBF,
    };
    static const uint8_t expected_query[] = {
        0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    };
    assert(m0601_build_drive_speed(1, 150, 0, 0, frame) == M0601_OK);
    assert(memcmp(frame, expected_positive_speed, sizeof(frame)) == 0);
    assert(m0601_build_drive_speed(1, 0, 0, 0, frame) == M0601_OK);
    assert(memcmp(frame, expected_zero, sizeof(frame)) == 0);
    assert(m0601_build_drive_speed(1, -150, 0, 0, frame) == M0601_OK);
    assert(memcmp(frame, expected_negative_speed, sizeof(frame)) == 0);
    assert(m0601_build_drive_position(1, 8192, 0, 0, frame) == M0601_OK);
    assert(memcmp(frame, expected_position_quarter, sizeof(frame)) == 0);
    assert(m0601_build_query(1, frame) == M0601_OK);
    assert(memcmp(frame, expected_query, sizeof(frame)) == 0);

    assert(m0601_build_set_mode(1, M0601_MODE_CURRENT, frame) == M0601_OK);
    assert(frame[0] == 1 && frame[1] == 0xA0 && frame[9] == M0601_MODE_CURRENT);
    assert(m0601_build_set_mode(1, M0601_MODE_SPEED, frame) == M0601_OK);
    assert(frame[9] == M0601_MODE_SPEED);
    assert(m0601_build_set_mode(1, M0601_MODE_POSITION, frame) == M0601_OK);
    assert(frame[9] == M0601_MODE_POSITION);

    assert(m0601_build_id_query(frame) == M0601_OK);
    static const uint8_t expected_id_query[] = {
        0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE,
    };
    assert(memcmp(frame, expected_id_query, sizeof(frame)) == 0);

    fake_transport_t fake = {0};
    const m0601_transport_t transport = {
        .ctx = &fake,
        .send = fake_send,
        .recv = NULL,
        .delay_ms = fake_delay,
    };
    m0601_motor_t motor;
    assert(m0601_init(&motor, &transport, 1, 80) == M0601_OK);
    assert(m0601_set_id(&motor, 2) == M0601_OK);
    assert(fake.sends == 5 && fake.delays == 4);
    assert(fake.last_frame[0] == 0xAA && fake.last_frame[1] == 0x55 &&
           fake.last_frame[2] == 0x53 && fake.last_frame[3] == 2);
}

int main(void)
{
    test_host_protocol();
    test_m0601_protocol();
    return 0;
}
