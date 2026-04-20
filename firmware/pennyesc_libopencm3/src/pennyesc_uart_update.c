#include "pennyesc_uart_update.h"
#include "pennyesc_boot.h"
#include <libopencm3/stm32/usart.h>
#include <string.h>

typedef struct {
    uint8_t buf[PNY_FRAME_MAX_PAYLOAD + 4u];
    uint8_t idx;
    uint8_t expected;
    uint32_t last_byte_ms;
} uart_update_parser_t;

static uart_update_parser_t parser;

static uint8_t crc8(const uint8_t *data, uint8_t len)
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

static void parser_reset(void)
{
    parser.idx = 0u;
    parser.expected = 0u;
}

#if defined(PNY_UART_UPDATE)
static void usart2_wait_tx_complete(void)
{
    while ((USART_ISR(USART2) & USART_ISR_TC) == 0u) {
    }
}
#endif

static void send_frame(uint8_t address, uint8_t cmd, const void *payload, uint8_t payload_len)
{
    uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u];

    frame[0] = PNY_FRAME_START;
    frame[1] = (uint8_t)((address << 4) | (cmd & 0x0Fu));
    frame[2] = payload_len;
    if (payload_len != 0u) {
        memcpy(&frame[3], payload, payload_len);
    }
    frame[3 + payload_len] = crc8(frame, (uint8_t)(3u + payload_len));

    for (uint8_t i = 0; i < (uint8_t)(4u + payload_len); i++) {
        usart_send_blocking(USART2, frame[i]);
    }
}

static bool parser_push(uint8_t byte, uint32_t now_ms, const uint8_t **frame, uint8_t *frame_len)
{
    if (parser.idx != 0u && (now_ms - parser.last_byte_ms) > 10u) {
        parser_reset();
    }
    parser.last_byte_ms = now_ms;

    if (parser.idx == 0u) {
        if (byte == PNY_FRAME_START) {
            parser.buf[parser.idx++] = byte;
        }
        return false;
    }

    if (parser.idx < 3u && byte == PNY_FRAME_START) {
        parser.buf[0] = byte;
        parser.idx = 1u;
        parser.expected = 0u;
        return false;
    }

    if (parser.idx >= sizeof(parser.buf)) {
        parser_reset();
        return false;
    }

    parser.buf[parser.idx++] = byte;
    if (parser.idx == 3u) {
        if (parser.buf[2] > PNY_FRAME_MAX_PAYLOAD) {
            parser_reset();
            return false;
        }
        parser.expected = (uint8_t)(parser.buf[2] + 4u);
    }

    if (parser.expected == 0u || parser.idx != parser.expected) {
        return false;
    }

    if (crc8(parser.buf, (uint8_t)(parser.expected - 1u)) != parser.buf[parser.expected - 1u]) {
        parser_reset();
        return false;
    }

    *frame = parser.buf;
    *frame_len = parser.expected;
    parser_reset();
    return true;
}

static bool handle_frame(
    const uint8_t *frame,
    uint8_t frame_len,
    uint8_t address,
    uint32_t now_ms,
    pennyesc_uart_update_fill_status_fn fill_status,
    pennyesc_uart_update_prepare_boot_fn prepare_boot
)
{
    uint8_t frame_address;
    uint8_t cmd;

    if (frame_len < 4u || frame[0] != PNY_FRAME_START) {
        return false;
    }

    frame_address = frame[1] >> 4;
    cmd = frame[1] & 0x0Fu;
    if (frame_address != address) {
        return false;
    }

    if (cmd == PNY_CMD_GET_STATUS) {
        pny_status_payload_t payload;
        memset(&payload, 0, sizeof(payload));
        if (fill_status != 0) {
            fill_status(&payload);
        }
        payload.result = PNY_RESULT_OK;
        send_frame(address, PNY_CMD_GET_STATUS, &payload, sizeof(payload));
        return true;
    }

    if (cmd != PNY_CMD_ENTER_BOOT) {
        return false;
    }

    if (frame[2] != 4u) {
        uint8_t result = PNY_RESULT_BAD_ARG;
        send_frame(address, PNY_CMD_ENTER_BOOT, &result, 1u);
        return true;
    }

    uint32_t magic = (uint32_t)frame[3] |
                     ((uint32_t)frame[4] << 8) |
                     ((uint32_t)frame[5] << 16) |
                     ((uint32_t)frame[6] << 24);
    if (magic != PNY_BOOT_MAGIC) {
        uint8_t result = PNY_RESULT_BAD_ARG;
        send_frame(address, PNY_CMD_ENTER_BOOT, &result, 1u);
        return true;
    }

#if defined(PNY_UART_UPDATE)
    uint8_t result = (prepare_boot == 0) ? PNY_RESULT_OK : prepare_boot();
    send_frame(address, PNY_CMD_ENTER_BOOT, &result, 1u);
    if (result == PNY_RESULT_OK) {
        usart2_wait_tx_complete();
        pennyesc_boot_request(now_ms);
    }
#else
    (void)now_ms;
    (void)prepare_boot;
    uint8_t result = PNY_RESULT_BAD_STATE;
    send_frame(address, PNY_CMD_ENTER_BOOT, &result, 1u);
#endif
    return true;
}

uint32_t pennyesc_uart_update_app_baud(uint32_t default_baud)
{
#if defined(PNY_UART_UPDATE)
    (void)default_baud;
    return PNY_UART_UPDATE_APP_BAUD;
#else
    return default_baud;
#endif
}

void pennyesc_uart_update_boot_window(volatile uint32_t *now_ms, uint8_t address)
{
#if defined(PNY_UART_UPDATE)
    uint32_t deadline = *now_ms + PNY_UART_UPDATE_BOOT_WINDOW_MS;

    parser_reset();
    while ((int32_t)(*now_ms - deadline) < 0) {
        const uint8_t *frame;
        uint8_t frame_len;

        if ((USART_ISR(USART2) & USART_ISR_RXNE) == 0u) {
            continue;
        }

        uint8_t byte = (uint8_t)USART_RDR(USART2);
        if (!parser_push(byte, *now_ms, &frame, &frame_len)) {
            continue;
        }

        if (handle_frame(frame, frame_len, address, *now_ms, 0, 0)) {
            pennyesc_boot_poll(*now_ms + 100u);
        }
    }
#else
    (void)now_ms;
    (void)address;
#endif
}

bool pennyesc_uart_update_feed_byte(
    uint8_t byte,
    uint8_t address,
    uint32_t now_ms,
    pennyesc_uart_update_fill_status_fn fill_status,
    pennyesc_uart_update_prepare_boot_fn prepare_boot
)
{
    const uint8_t *frame;
    uint8_t frame_len;

    if (!parser_push(byte, now_ms, &frame, &frame_len)) {
        return false;
    }
    return handle_frame(frame, frame_len, address, now_ms, fill_status, prepare_boot);
}

void pennyesc_uart_update_poll(uint32_t now_ms)
{
    pennyesc_boot_poll(now_ms);
}
