#include "common/crc16.h"

uint16_t robot_crc16_ccitt_false(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFu;

    if (data == NULL) {
        return crc;
    }

    for (size_t index = 0; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000u) != 0u
                      ? (uint16_t)((crc << 1) ^ 0x1021u)
                      : (uint16_t)(crc << 1);
        }
    }

    return crc;
}
