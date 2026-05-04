#ifndef PENNYESC_FRAME_H
#define PENNYESC_FRAME_H

#include <stdbool.h>
#include <stdint.h>
#include "pennyesc_protocol.h"

#define PNY_FRAME_BUF_SIZE (PNY_FRAME_MAX_PAYLOAD + 4u)

typedef struct {
    uint8_t buf[PNY_FRAME_BUF_SIZE];
    uint8_t idx;
    uint8_t expected;
    bool crc_error;
    uint32_t last_byte_ms;
} pny_frame_parser_t;

uint8_t pny_frame_crc8(const uint8_t *data, uint8_t len);
void pny_frame_parser_reset(pny_frame_parser_t *parser);
bool pny_frame_parser_push(
    pny_frame_parser_t *parser,
    uint8_t byte,
    uint32_t now_ms,
    uint8_t timeout_ms,
    const uint8_t **frame,
    uint8_t *frame_len
);
void pny_frame_send(uint8_t header, const void *payload, uint8_t payload_len);

#endif
