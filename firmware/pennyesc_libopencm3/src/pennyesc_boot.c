#include "pennyesc_boot.h"
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>

#define PENNYESC_BOOT_DELAY_MS 20u

#if defined(PNY_UART_UPDATE)
static volatile uint32_t pending_boot_ms;

void pennyesc_boot_request(uint32_t now_ms)
{
    pending_boot_ms = now_ms + PENNYESC_BOOT_DELAY_MS;
}

void pennyesc_boot_poll(uint32_t now_ms)
{
    if (pending_boot_ms == 0u || (int32_t)(now_ms - pending_boot_ms) < 0) {
        return;
    }
    while ((USART_ISR(USART2) & USART_ISR_TC) == 0u) {
    }
    /* The caller has stopped the motor. System reset resets every peripheral,
     * including both scheduler timers, DMA, and the interrupt controller. */
    scb_reset_system();
}
#else
void pennyesc_boot_request(uint32_t now_ms)
{
    (void)now_ms;
}

void pennyesc_boot_poll(uint32_t now_ms)
{
    (void)now_ms;
}
#endif
