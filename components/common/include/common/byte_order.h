#ifndef ROBOT_COMMON_BYTE_ORDER_H
#define ROBOT_COMMON_BYTE_ORDER_H

#include <stdint.h>

static inline uint16_t robot_read_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static inline int16_t robot_read_i16_le(const uint8_t *src)
{
    return (int16_t)robot_read_u16_le(src);
}

static inline uint32_t robot_read_u32_le(const uint8_t *src)
{
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

static inline void robot_write_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)(value >> 8);
}

static inline void robot_write_i16_le(uint8_t *dst, int16_t value)
{
    robot_write_u16_le(dst, (uint16_t)value);
}

static inline void robot_write_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFu);
    dst[1] = (uint8_t)((value >> 8) & 0xFFu);
    dst[2] = (uint8_t)((value >> 16) & 0xFFu);
    dst[3] = (uint8_t)(value >> 24);
}

#endif
