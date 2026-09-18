#include "pennyesc_frame.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static __attribute__((noinline)) void uart_tx_start(void)
{
    gpio_set(GPIOA, GPIO9);
    /* PA9's alternate function and drive strength are configured at startup. */
    GPIO_PUPDR(GPIOA) &= ~(3u << 18);
    GPIO_MODER(GPIOA) = (GPIO_MODER(GPIOA) & ~(3u << 18)) | (GPIO_MODE_AF << 18);
    USART_ICR(USART2) = USART_ICR_TCCF;
    USART_CR1(USART2) |= USART_CR1_TE;
    while ((USART_ISR(USART2) & USART_ISR_TEACK) == 0u) {
    }
}

static __attribute__((noinline)) void uart_tx_stop(void)
{
    while ((USART_ISR(USART2) & USART_ISR_TC) == 0u) {
    }
    USART_CR1(USART2) &= ~USART_CR1_TE;
    while ((USART_ISR(USART2) & USART_ISR_TEACK) != 0u) {
    }
    GPIO_MODER(GPIOA) &= ~(3u << 18);
    GPIO_PUPDR(GPIOA) = (GPIO_PUPDR(GPIOA) & ~(3u << 18)) | (GPIO_PUPD_PULLUP << 18);
}

static uint8_t crc8_byte(uint8_t crc, uint8_t byte)
{
    crc ^= byte;
    for (uint8_t bit = 0; bit < 8; bit++) {
        crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}

uint8_t pny_frame_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8_byte(crc, data[i]);
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
    uint32_t prefix = PNY_FRAME_START | ((uint32_t)header << 8) | ((uint32_t)payload_len << 16);
    const uint8_t *bytes = payload;
    uint8_t crc = 0u;

    /* Stream the frame: a second 68-byte stack buffer leaves too little room
     * for nested sensor and commutation interrupts on the 2 KB MCU. */
    uart_tx_start();
    for (uint8_t i = 0; i < (uint8_t)(3u + payload_len); i++) {
        uint8_t byte = i < 3u ? (uint8_t)(prefix >> (8u * i)) : bytes[i - 3u];
        crc = crc8_byte(crc, byte);
        usart_send_blocking(USART2, byte);
    }
    usart_send_blocking(USART2, crc);
    uart_tx_stop();
}
