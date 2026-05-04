#include "pennyesc_frame.h"
#include <libopencm3/stm32/usart.h>
#include <string.h>

uint8_t pny_frame_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }

    return crc;
}

void pny_frame_parser_reset(pny_frame_parser_t *parser)
{
    parser->idx = 0u;
    parser->expected = 0u;
}

bool pny_frame_parser_push(
    pny_frame_parser_t *parser,
    uint8_t byte,
    uint32_t now_ms,
    uint8_t timeout_ms,
    const uint8_t **frame,
    uint8_t *frame_len
)
{
    parser->crc_error = false;

    if (parser->idx != 0u && (now_ms - parser->last_byte_ms) > timeout_ms) {
        pny_frame_parser_reset(parser);
    }
    parser->last_byte_ms = now_ms;

    if (parser->idx == 0u) {
        if (byte == PNY_FRAME_START) {
            parser->buf[parser->idx++] = byte;
        }
        return false;
    }

    if (parser->idx < 3u && byte == PNY_FRAME_START) {
        parser->buf[0] = byte;
        parser->idx = 1u;
        parser->expected = 0u;
        return false;
    }

    if (parser->idx >= sizeof(parser->buf)) {
        pny_frame_parser_reset(parser);
        return false;
    }

    parser->buf[parser->idx++] = byte;
    if (parser->idx == 3u) {
        if (parser->buf[2] > PNY_FRAME_MAX_PAYLOAD) {
            pny_frame_parser_reset(parser);
            return false;
        }
        parser->expected = (uint8_t)(parser->buf[2] + 4u);
    }

    if (parser->expected == 0u || parser->idx != parser->expected) {
        return false;
    }

    if (pny_frame_crc8(parser->buf, (uint8_t)(parser->expected - 1u)) != parser->buf[parser->expected - 1u]) {
        parser->crc_error = true;
        pny_frame_parser_reset(parser);
        return false;
    }

    *frame = parser->buf;
    *frame_len = parser->expected;
    pny_frame_parser_reset(parser);
    return true;
}

void pny_frame_send(uint8_t header, const void *payload, uint8_t payload_len)
{
    uint8_t frame[PNY_FRAME_BUF_SIZE];

    frame[0] = PNY_FRAME_START;
    frame[1] = header;
    frame[2] = payload_len;
    if (payload_len != 0u) {
        memcpy(&frame[3], payload, payload_len);
    }
    frame[3 + payload_len] = pny_frame_crc8(frame, (uint8_t)(3u + payload_len));

    for (uint8_t i = 0; i < (uint8_t)(4u + payload_len); i++) {
        usart_send_blocking(USART2, frame[i]);
    }
}
