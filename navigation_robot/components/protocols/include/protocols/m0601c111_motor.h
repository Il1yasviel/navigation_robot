#ifndef M0601C111_MOTOR_H
#define M0601C111_MOTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define M0601_FRAME_SIZE             10u
#define M0601_RECOMMENDED_BAUDRATE   115200u
#define M0601_ID_QUERY_ADDRESS       0xC8u
#define M0601_CMD_DRIVE              0x64u
#define M0601_CMD_QUERY              0x74u
#define M0601_CMD_SET_MODE           0xA0u
#define M0601_SET_ID_HEAD0           0xAAu
#define M0601_SET_ID_HEAD1           0x55u
#define M0601_SET_ID_HEAD2           0x53u
#define M0601_SPEED_RPM_MIN          (-330)
#define M0601_SPEED_RPM_MAX          330
#define M0601_CURRENT_RAW_MIN        (-32767)
#define M0601_CURRENT_RAW_MAX        32767
#define M0601_POSITION_RAW_MAX       32767u
#define M0601_ACCEL_DEFAULT          0u
#define M0601_BRAKE_OFF              0x00u
#define M0601_BRAKE_ON               0xFFu
#define M0601_MODE_CURRENT           0x01u
#define M0601_MODE_SPEED             0x02u
#define M0601_MODE_POSITION          0x03u
#define M0601_SET_ID_REPEAT_COUNT    5u

typedef enum {
    M0601_OK = 0,
    M0601_ERROR_NULL = -1,
    M0601_ERROR_RANGE = -2,
    M0601_ERROR_IO = -3,
    M0601_ERROR_TIMEOUT = -4,
    M0601_ERROR_CRC = -5,
    M0601_ERROR_FRAME = -6
} m0601_status_t;

typedef enum {
    M0601_FAULT_SENSOR            = 1u << 0,
    M0601_FAULT_OVERCURRENT       = 1u << 1,
    M0601_FAULT_PHASE_OVERCURRENT = 1u << 2,
    M0601_FAULT_STALL             = 1u << 3,
    M0601_FAULT_OVERTEMP          = 1u << 4
} m0601_fault_t;

typedef m0601_status_t (*m0601_send_fn)(void *, const uint8_t *, size_t, uint32_t);
typedef m0601_status_t (*m0601_recv_fn)(void *, uint8_t *, size_t, uint32_t);
typedef void (*m0601_delay_ms_fn)(void *, uint32_t);

typedef struct {
    void *ctx;
    m0601_send_fn send;
    m0601_recv_fn recv;
    m0601_delay_ms_fn delay_ms;
} m0601_transport_t;

typedef struct {
    m0601_transport_t transport;
    uint8_t default_id;
    uint32_t timeout_ms;
} m0601_motor_t;

typedef struct {
    uint8_t id;
    uint8_t mode;
    int16_t torque_current_raw;
    int16_t speed_rpm;
    uint16_t position_raw;
    uint8_t fault;
} m0601_drive_feedback_t;

typedef struct {
    uint8_t id;
    uint8_t mode;
    int16_t torque_current_raw;
    int16_t speed_rpm;
    uint8_t temperature_raw;
    uint8_t position_u8;
    uint8_t fault;
} m0601_query_feedback_t;

m0601_status_t m0601_init(m0601_motor_t *, const m0601_transport_t *, uint8_t, uint32_t);
uint8_t m0601_crc8_maxim(const uint8_t *, size_t);
bool m0601_check_crc(const uint8_t frame[M0601_FRAME_SIZE]);
m0601_status_t m0601_build_drive_raw(uint8_t, int16_t, uint8_t, uint8_t, uint8_t *);
m0601_status_t m0601_build_drive_current(uint8_t, int16_t, uint8_t, uint8_t, uint8_t *);
m0601_status_t m0601_build_drive_speed(uint8_t, int16_t, uint8_t, uint8_t, uint8_t *);
m0601_status_t m0601_build_drive_position(uint8_t, uint16_t, uint8_t, uint8_t, uint8_t *);
m0601_status_t m0601_build_query(uint8_t, uint8_t *);
m0601_status_t m0601_build_id_query(uint8_t *);
m0601_status_t m0601_build_set_mode(uint8_t, uint8_t, uint8_t *);
m0601_status_t m0601_build_set_id(uint8_t, uint8_t *);
m0601_status_t m0601_parse_drive_feedback(const uint8_t *, m0601_drive_feedback_t *);
m0601_status_t m0601_parse_query_feedback(const uint8_t *, m0601_query_feedback_t *);
m0601_status_t m0601_drive_raw(m0601_motor_t *, uint8_t, int16_t, uint8_t, uint8_t, m0601_drive_feedback_t *);
m0601_status_t m0601_drive_default(m0601_motor_t *, int16_t, uint8_t, uint8_t, m0601_drive_feedback_t *);
m0601_status_t m0601_drive_current(m0601_motor_t *, uint8_t, int16_t, uint8_t, uint8_t, m0601_drive_feedback_t *);
m0601_status_t m0601_drive_speed(m0601_motor_t *, uint8_t, int16_t, uint8_t, uint8_t, m0601_drive_feedback_t *);
m0601_status_t m0601_drive_position(m0601_motor_t *, uint8_t, uint16_t, uint8_t, uint8_t, m0601_drive_feedback_t *);
m0601_status_t m0601_query(m0601_motor_t *, uint8_t, m0601_query_feedback_t *);
m0601_status_t m0601_query_default(m0601_motor_t *, m0601_query_feedback_t *);
m0601_status_t m0601_query_id(m0601_motor_t *, m0601_drive_feedback_t *);
m0601_status_t m0601_set_mode(m0601_motor_t *, uint8_t, uint8_t);
m0601_status_t m0601_set_id(m0601_motor_t *, uint8_t);

#ifdef __cplusplus
}
#endif
#endif
