#include "pennyesc_uart_update.h"
#include "pennyesc_boot.h"
#include "pennyesc_frame.h"
#include <libopencm3/stm32/usart.h>
#include <string.h>

static pny_frame_parser_t parser;
static const uint8_t boot_sync = 0x7Fu;

#if defined(PNY_UART_UPDATE)
static void usart2_wait_tx_complete(void)
{
    while ((USART_ISR(USART2) & USART_ISR_TC) == 0u) {
    }
}
#endif

static uint8_t frame_header(uint8_t address, uint8_t cmd)
{
    return (uint8_t)((address << 4) | (cmd & 0x0Fu));
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
        pny_frame_send(frame_header(address, PNY_CMD_GET_STATUS), &payload, sizeof(payload));
        return true;
    }

    if (cmd != PNY_CMD_ENTER_BOOT) {
        return false;
    }

    if (frame[2] != 4u) {
        uint8_t result = PNY_RESULT_BAD_ARG;
        pny_frame_send(frame_header(address, PNY_CMD_ENTER_BOOT), &result, 1u);
        return true;
    }

    uint32_t magic = (uint32_t)frame[3] |
                     ((uint32_t)frame[4] << 8) |
                     ((uint32_t)frame[5] << 16) |
                     ((uint32_t)frame[6] << 24);
    if (magic != PNY_BOOT_MAGIC) {
        uint8_t result = PNY_RESULT_BAD_ARG;
        pny_frame_send(frame_header(address, PNY_CMD_ENTER_BOOT), &result, 1u);
        return true;
    }

#if defined(PNY_UART_UPDATE)
    uint8_t result = (prepare_boot == 0) ? PNY_RESULT_OK : prepare_boot();
    pny_frame_send(frame_header(address, PNY_CMD_ENTER_BOOT), &result, 1u);
    if (result == PNY_RESULT_OK) {
        usart2_wait_tx_complete();
        pennyesc_boot_request(now_ms);
    }
#else
    (void)now_ms;
    (void)prepare_boot;
    uint8_t result = PNY_RESULT_BAD_STATE;
    pny_frame_send(frame_header(address, PNY_CMD_ENTER_BOOT), &result, 1u);
#endif
    return true;
}

uint32_t pennyesc_uart_update_app_baud(uint32_t default_baud)
{
#if defined(PNY_UART_UPDATE)
#if defined(PNY_UART_UPDATE_APP_BAUD)
    return PNY_UART_UPDATE_APP_BAUD;
#else
    return default_baud;
#endif
#else
    return default_baud;
#endif
}

void pennyesc_uart_update_boot_window(volatile uint32_t *now_ms, uint8_t address)
{
#if defined(PNY_UART_UPDATE)
    uint32_t deadline = *now_ms + PNY_UART_UPDATE_BOOT_WINDOW_MS;

    pny_frame_parser_reset(&parser);
    while ((int32_t)(*now_ms - deadline) < 0) {
        const uint8_t *frame;
        uint8_t frame_len;

        if ((USART_ISR(USART2) & USART_ISR_RXNE) == 0u) {
            continue;
        }

        uint8_t byte = (uint8_t)USART_RDR(USART2);
        if (byte == boot_sync) {
            pennyesc_boot_request(*now_ms);
            pennyesc_boot_poll(*now_ms + 100u);
            continue;
        }
        if (!pny_frame_parser_push(&parser, byte, *now_ms, 10u, &frame, &frame_len)) {
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

    if (!pny_frame_parser_push(&parser, byte, now_ms, 10u, &frame, &frame_len)) {
        return false;
    }
    return handle_frame(frame, frame_len, address, now_ms, fill_status, prepare_boot);
}

void pennyesc_uart_update_poll(uint32_t now_ms)
{
    pennyesc_boot_poll(now_ms);
}
