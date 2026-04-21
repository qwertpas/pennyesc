#ifndef PENNYESC_UART_UPDATE_H
#define PENNYESC_UART_UPDATE_H

#include <stdbool.h>
#include <stdint.h>
#include "pennyesc_protocol.h"

#ifndef PNY_UART_UPDATE_BOOT_BAUD
#define PNY_UART_UPDATE_BOOT_BAUD 115200u
#endif

#ifndef PNY_UART_UPDATE_BOOT_WINDOW_MS
#define PNY_UART_UPDATE_BOOT_WINDOW_MS 5000u
#endif

typedef void (*pennyesc_uart_update_fill_status_fn)(pny_status_payload_t *payload);
typedef uint8_t (*pennyesc_uart_update_prepare_boot_fn)(void);

uint32_t pennyesc_uart_update_app_baud(uint32_t default_baud);
void pennyesc_uart_update_boot_window(volatile uint32_t *now_ms, uint8_t address);
bool pennyesc_uart_update_feed_byte(
    uint8_t byte,
    uint8_t address,
    uint32_t now_ms,
    pennyesc_uart_update_fill_status_fn fill_status,
    pennyesc_uart_update_prepare_boot_fn prepare_boot
);
void pennyesc_uart_update_poll(uint32_t now_ms);

#endif
