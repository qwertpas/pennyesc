#include "pennyesc_boot.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#define PENNYESC_FLASH_BASE 0x08000000u
#define PENNYESC_BOOT_BASE 0x1FF00000u
#define PENNYESC_BOOT_DELAY_MS 20u

#if defined(PNY_UART_UPDATE)

static volatile uint32_t pending_boot_ms;

static void __attribute__((noinline, long_call, section(".ramtext")))
pennyesc_boot_handoff_from_ram(void)
{
    while ((FLASH_SR & FLASH_SR_BSY) != 0u) {
    }

    FLASH_PEKEYR = FLASH_PEKEYR_PEKEY1;
    FLASH_PEKEYR = FLASH_PEKEYR_PEKEY2;
    FLASH_PRGKEYR = FLASH_PRGKEYR_PRGKEY1;
    FLASH_PRGKEYR = FLASH_PRGKEYR_PRGKEY2;

    FLASH_PECR |= FLASH_PECR_ERASE | FLASH_PECR_PROG;
    MMIO32(PENNYESC_FLASH_BASE) = 0u;
    while ((FLASH_SR & FLASH_SR_BSY) != 0u) {
    }
    FLASH_PECR &= ~(FLASH_PECR_ERASE | FLASH_PECR_PROG);

    uint32_t stack = MMIO32(PENNYESC_BOOT_BASE + 0u);
    uint32_t reset = MMIO32(PENNYESC_BOOT_BASE + 4u);

    SCB_VTOR = PENNYESC_BOOT_BASE;
    __asm__ volatile("msr msp, %0" : : "r"(stack) :);
    ((void (*)(void))reset)();

    while (1) {
    }
}

void pennyesc_boot_request(uint32_t now_ms)
{
    pending_boot_ms = now_ms + PENNYESC_BOOT_DELAY_MS;
}

void pennyesc_boot_poll(uint32_t now_ms)
{
    if (pending_boot_ms == 0u || (int32_t)(now_ms - pending_boot_ms) < 0) {
        return;
    }

    pending_boot_ms = 0u;

    while ((USART_ISR(USART2) & USART_ISR_TC) == 0u) {
    }

    __asm__("cpsid i");
    systick_interrupt_disable();
    systick_counter_disable();

    timer_disable_counter(TIM21);
    timer_disable_irq(TIM21, TIM_DIER_UIE);
    nvic_disable_irq(NVIC_TIM21_IRQ);

    timer_disable_counter(TIM2);

    usart_disable_rx_dma(USART2);
    dma_disable_channel(DMA1, DMA_CHANNEL5);
    usart_disable(USART2);

    i2c_peripheral_disable(I2C1);
    spi_disable(SPI1);

    pennyesc_boot_handoff_from_ram();
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
