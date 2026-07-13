#ifndef ROBOT_HOST_MESSAGES_H
#define ROBOT_HOST_MESSAGES_H

#include <stdint.h>

#define HOST_MSG_HELLO             0x01u
#define HOST_MSG_SET_SINGLE_RPM    0x10u
#define HOST_MSG_JOYSTICK          0x11u
#define HOST_MSG_STOP              0x12u
#define HOST_MSG_QUERY_MOTOR       0x13u
#define HOST_MSG_QUERY_UNIQUE_ID   0x14u
#define HOST_MSG_SET_ID            0x15u
#define HOST_MSG_SET_MODE          0x16u
#define HOST_MSG_SET_CURRENT       0x17u
#define HOST_MSG_SET_POSITION      0x18u
#define HOST_MSG_CONTROL_KEEPALIVE 0x19u
#define HOST_MSG_ACK               0x80u
#define HOST_MSG_HEARTBEAT         0x90u
#define HOST_MSG_EVENT             0x91u

#define HOST_FLAG_ACK_REQUIRED     0x01u
#define HOST_SET_ID_CONFIRM        0x4D36u
#define HOST_HEARTBEAT_SIZE        30u

typedef enum {
    HOST_STATUS_OK = 0,
    HOST_STATUS_BAD_CRC = 1,
    HOST_STATUS_BAD_LENGTH = 2,
    HOST_STATUS_RANGE = 3,
    HOST_STATUS_BUSY = 4,
    HOST_STATUS_MOTOR_TIMEOUT = 5,
    HOST_STATUS_MOTOR_CRC = 6,
    HOST_STATUS_PRECONDITION = 7,
    HOST_STATUS_UNSUPPORTED = 8,
    HOST_STATUS_IO = 9
} host_status_t;

typedef enum {
    MOTOR_STATE_OFFLINE = 0,
    MOTOR_STATE_IDLE = 1,
    MOTOR_STATE_RUNNING = 2,
    MOTOR_STATE_FAULT = 3,
    MOTOR_STATE_ESTOP = 4
} host_motor_state_t;

#endif
