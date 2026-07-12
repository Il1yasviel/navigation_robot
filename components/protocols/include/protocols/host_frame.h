#ifndef ROBOT_HOST_FRAME_H
#define ROBOT_HOST_FRAME_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HOST_FRAME_SOF0          0xAAu
#define HOST_FRAME_SOF1          0x55u
#define HOST_FRAME_VERSION       0x01u
#define HOST_FRAME_MAX_PAYLOAD   128u
#define HOST_FRAME_OVERHEAD      10u
#define HOST_FRAME_MAX_SIZE      (HOST_FRAME_MAX_PAYLOAD + HOST_FRAME_OVERHEAD)

typedef struct {
    uint8_t type;
    uint8_t sequence;
    uint8_t flags;
    uint16_t payload_length;
    uint8_t payload[HOST_FRAME_MAX_PAYLOAD];
} host_frame_t;

typedef void (*host_frame_callback_t)(void *context, const host_frame_t *frame);

typedef struct {
    uint8_t buffer[HOST_FRAME_MAX_SIZE];
    size_t buffered;
    uint32_t valid_frames;
    uint32_t crc_errors;
    uint32_t length_errors;
} host_stream_parser_t;

void host_stream_parser_init(host_stream_parser_t *parser);
void host_stream_parser_feed(host_stream_parser_t *parser,
                             const uint8_t *data,
                             size_t length,
                             host_frame_callback_t callback,
                             void *context);
size_t host_frame_encode(uint8_t type,
                         uint8_t sequence,
                         uint8_t flags,
                         const uint8_t *payload,
                         uint16_t payload_length,
                         uint8_t *output,
                         size_t output_capacity);

#ifdef __cplusplus
}
#endif

#endif
