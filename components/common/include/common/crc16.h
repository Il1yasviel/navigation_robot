#ifndef ROBOT_COMMON_CRC16_H
#define ROBOT_COMMON_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t robot_crc16_ccitt_false(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
