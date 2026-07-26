#include "protocols/host_frame.h"

#include <string.h>

#include "common/byte_order.h"
#include "common/crc16.h"

static void parser_discard(host_stream_parser_t *parser, size_t count)
{
    if (count >= parser->buffered) {
        parser->buffered = 0;
        return;
    }

    (void)memmove(parser->buffer,
                  &parser->buffer[count],
                  parser->buffered - count);
    parser->buffered -= count;
}

static void parser_process(host_stream_parser_t *parser,
                           host_frame_callback_t callback,
                           void *context)
{
    while (parser->buffered >= 2u) {
        size_t sof = 0;
        while ((sof + 1u) < parser->buffered &&
               (parser->buffer[sof] != HOST_FRAME_SOF0 ||
                parser->buffer[sof + 1u] != HOST_FRAME_SOF1)) {
            ++sof;
        }

        if (sof > 0u) {
            parser_discard(parser, sof);
        }
        if (parser->buffered < 2u) {
            return;
        }
        if (parser->buffer[0] != HOST_FRAME_SOF0 ||
            parser->buffer[1] != HOST_FRAME_SOF1) {
            parser_discard(parser, 1u);
            continue;
        }
        if (parser->buffered < 8u) {
            return;
        }
        if (parser->buffer[2] != HOST_FRAME_VERSION) {
            parser_discard(parser, 1u);
            continue;
        }

        const uint16_t payload_length = robot_read_u16_le(&parser->buffer[6]);
        if (payload_length > HOST_FRAME_MAX_PAYLOAD) {
            ++parser->length_errors;
            parser_discard(parser, 1u);
            continue;
        }

        const size_t frame_size = HOST_FRAME_OVERHEAD + payload_length;
        if (parser->buffered < frame_size) {
            return;
        }

        const uint16_t received_crc =
            robot_read_u16_le(&parser->buffer[8u + payload_length]);
        const uint16_t expected_crc =
            robot_crc16_ccitt_false(&parser->buffer[2], 6u + payload_length);
        if (received_crc != expected_crc) {
            ++parser->crc_errors;
            parser_discard(parser, 1u);
            continue;
        }

        host_frame_t frame = {
            .type = parser->buffer[3],
            .sequence = parser->buffer[4],
            .flags = parser->buffer[5],
            .payload_length = payload_length,
        };
        if (payload_length > 0u) {
            (void)memcpy(frame.payload, &parser->buffer[8], payload_length);
        }

        ++parser->valid_frames;
        if (callback != NULL) {
            callback(context, &frame);
        }
        parser_discard(parser, frame_size);
    }
}

void host_stream_parser_init(host_stream_parser_t *parser)
{
    if (parser != NULL) {
        (void)memset(parser, 0, sizeof(*parser));
    }
}

void host_stream_parser_feed(host_stream_parser_t *parser,
                             const uint8_t *data,
                             size_t length,
                             host_frame_callback_t callback,
                             void *context)
{
    if (parser == NULL || data == NULL) {
        return;
    }

    for (size_t index = 0; index < length; ++index) {
        if (parser->buffered == sizeof(parser->buffer)) {
            parser_discard(parser, 1u);
        }
        parser->buffer[parser->buffered++] = data[index];
        parser_process(parser, callback, context);
    }
}

size_t host_frame_encode(uint8_t type,
                         uint8_t sequence,
                         uint8_t flags,
                         const uint8_t *payload,
                         uint16_t payload_length,
                         uint8_t *output,
                         size_t output_capacity)
{
    const size_t total = HOST_FRAME_OVERHEAD + payload_length;
    if (output == NULL || payload_length > HOST_FRAME_MAX_PAYLOAD ||
        output_capacity < total || (payload_length > 0u && payload == NULL)) {
        return 0u;
    }

    output[0] = HOST_FRAME_SOF0;
    output[1] = HOST_FRAME_SOF1;
    output[2] = HOST_FRAME_VERSION;
    output[3] = type;
    output[4] = sequence;
    output[5] = flags;
    robot_write_u16_le(&output[6], payload_length);
    if (payload_length > 0u) {
        (void)memcpy(&output[8], payload, payload_length);
    }

    const uint16_t crc = robot_crc16_ccitt_false(&output[2], 6u + payload_length);
    robot_write_u16_le(&output[8u + payload_length], crc);
    return total;
}
