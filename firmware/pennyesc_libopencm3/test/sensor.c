/* Mock only the hardware registers; exercise the production sensor state machine. */
#include <assert.h>
#include <stdint.h>
#include "tmag5273.h"
#define I2C1 0
#define TIM21 0
#define NVIC_I2C1_IRQ 0
static uint32_t cr1, cr2, icr, hw_isr, txdr, rxdr;
#define I2C_CR1(t) cr1
#define I2C_CR2(t) cr2
#define I2C_ICR(t) icr
#define I2C_ISR(t) hw_isr
#define I2C_TXDR(t) txdr
#define I2C_RXDR(t) rxdr
#define I2C_CR1_ERRIE 1u
#define I2C_CR1_NACKIE 2u
#define I2C_CR1_STOPIE 4u
#define I2C_CR1_TCIE 8u
#define I2C_CR1_RXDMAEN 16u
#define I2C_ISR_TXE 1u
#define I2C_ISR_NACKF 2u
#define I2C_ISR_BERR 4u
#define I2C_ISR_ARLO 8u
#define I2C_ISR_OVR 16u
#define I2C_ISR_TIMEOUT 32u
#define I2C_ISR_TC 64u
#define I2C_ISR_STOPF 128u
#define I2C_ISR_BUSY 256u
#define I2C_ICR_STOPCF 1u
#define I2C_ICR_NACKCF 2u
#define I2C_ICR_BERRCF 4u
#define I2C_ICR_ARLOCF 8u
#define I2C_ICR_OVRCF 16u
#define I2C_ICR_TIMOUTCF 32u
#define I2C_CR2_SADD_7BIT_SHIFT 1
#define I2C_CR2_NBYTES_SHIFT 16
#define I2C_CR2_RD_WRN (1u << 10)
#define I2C_CR2_AUTOEND (1u << 25)
#define I2C_CR2_START (1u << 13)
#define DMA1 0
#define DMA_CHANNEL3 3
#define DMA_CCR_MINC 128u
#define DMA_CCR_EN 1u
static uint32_t dma_ccr, dma_cpar, dma_cmar, dma_cndtr;
#define DMA_CCR(d, c) dma_ccr
#define DMA_CPAR(d, c) dma_cpar
#define DMA_CMAR(d, c) dma_cmar
#define DMA_CNDTR(d, c) dma_cndtr
static uint16_t clock_tick;
static unsigned timer_get_counter(int timer) { (void)timer; return clock_tick; }
static void nvic_disable_irq(int irq) { (void)irq; }
static void nvic_enable_irq(int irq) { (void)irq; }
/* STATE */
static void i2c1_recover(void) { tmag_stats.recover_count++; hw_isr = 0; }
/* FUNCTIONS */

/* One trigger-and-read transaction delivering (x, y); returns true if the
 * driver produced a sample. dma_left simulates an incomplete DMA transfer. */
static bool transaction(int16_t x, int16_t y, unsigned dma_left) {
    clock_tick += 125;
    assert(async_state == ASYNC_IDLE);
    assert(tmag5273_async_start_xy(clock_tick));
    assert(!tmag5273_async_start_xy(clock_tick + 1));   // busy
    assert(async_state == ASYNC_TRIGGER && txdr == 0x92u);
    assert(!(cr2 & I2C_CR2_RD_WRN) && ((cr2 >> 16) & 0xff) == 1 && !(cr2 & I2C_CR2_AUTOEND));
    assert(cr1 & I2C_CR1_RXDMAEN);
    hw_isr = I2C_ISR_TC;
    assert(!tmag5273_i2c1_isr());   // trigger sent, read started
    assert((cr2 & I2C_CR2_RD_WRN) && ((cr2 >> 16) & 0xff) == 4 && (cr2 & I2C_CR2_AUTOEND));
    assert((dma_ccr & DMA_CCR_EN) && dma_cndtr == 4);
    async_rx[0] = (uint8_t)((uint16_t)x >> 8); async_rx[1] = (uint8_t)x;
    async_rx[2] = (uint8_t)((uint16_t)y >> 8); async_rx[3] = (uint8_t)y;
    dma_cndtr = dma_left;
    hw_isr = I2C_ISR_STOPF;
    assert(tmag5273_i2c1_isr());
    hw_isr = 0;
    assert(async_state == ASYNC_IDLE && dma_ccr == 0 && !(cr1 & I2C_CR1_RXDMAEN));
    return async_sample_ready;
}

int main(void) {
    tmag5273_xy_sample_t sample;
    clock_tick = 65400;
    // The first read returns a conversion of unknown age.
    assert(!transaction(100, 200, 0));
    uint16_t first = clock_tick;
    // The next returns the conversion the first transaction triggered.
    assert(transaction(0x1234, -2, 0));
    assert(tmag5273_async_take_xy(&sample));
    assert(sample.x == 0x1234 && sample.y == -2 && sample.z == 0);
    assert(sample.sample_tick == first);
    assert(tmag_stats.sample_count == 1);
    assert(!tmag5273_async_take_xy(&sample));
    uint16_t prior = clock_tick;
    assert(transaction(0x1235, -2, 0));
    assert(tmag5273_async_take_xy(&sample) && sample.sample_tick == prior);
    // A repeated result is stale, and the conversion was still running, so
    // the following result's trigger time is unknown too.
    assert(!transaction(0x1235, -2, 0));
    assert(!transaction(0x2000, 7, 0));
    prior = clock_tick;
    assert(transaction(0x2001, 7, 0));
    assert(tmag5273_async_take_xy(&sample) && sample.sample_tick == prior && sample.x == 0x2001);
    // An incomplete DMA transfer is discarded and also breaks the chain.
    assert(!transaction(0x2100, 8, 1));
    assert(!transaction(0x2200, 9, 0));
    assert(transaction(0x2300, 10, 0));
    assert(tmag5273_async_take_xy(&sample));
    // NACK: recover, and the next result's age is unknown.
    clock_tick += 125;
    assert(tmag5273_async_start_xy(clock_tick));
    hw_isr = I2C_ISR_NACKF;
    assert(tmag5273_i2c1_isr());
    assert(tmag_stats.nack_count == 1 && tmag_stats.recover_count == 1);
    assert(async_state == ASYNC_IDLE);
    assert(!transaction(0x2400, 11, 0));
    assert(transaction(0x2500, 12, 0));
    assert(tmag5273_async_take_xy(&sample));
    // Cancel drops a pending sample and the trigger chain.
    assert(transaction(0x2600, 13, 0));
    tmag5273_async_cancel();
    assert(async_state == ASYNC_IDLE && !async_sample_ready);
    assert(!transaction(0x2700, 14, 0));
    return 0;
}
