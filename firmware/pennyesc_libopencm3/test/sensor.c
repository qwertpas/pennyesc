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
#define I2C_CR1_RXIE 16u
#define I2C_CR1_TXIE 32u
#define I2C_ISR_NACKF 1u
#define I2C_ISR_BERR 2u
#define I2C_ISR_ARLO 4u
#define I2C_ISR_OVR 8u
#define I2C_ISR_TIMEOUT 16u
#define I2C_ISR_TXIS 32u
#define I2C_ISR_RXNE 64u
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
static uint16_t clock_tick;
static unsigned timer_get_counter(int timer) { (void)timer; return clock_tick; }
static void nvic_disable_irq(int irq) { (void)irq; }
static void nvic_enable_irq(int irq) { (void)irq; }
/* STATE */
static void i2c1_recover(void) { tmag_stats.recover_count++; hw_isr = 0; }
/* FUNCTIONS */
static void result(uint8_t status) {
    uint8_t bytes[] = {0x12, 0xff, status};
    for (unsigned i = 0; i < sizeof(bytes); ++i) {
        rxdr = bytes[i]; hw_isr = I2C_ISR_RXNE; tmag5273_i2c1_isr();
    }
    hw_isr = I2C_ISR_STOPF; tmag5273_i2c1_isr(); hw_isr = 0;
}
int main(void) {
    clock_tick = 65500;
    assert(tmag5273_async_start_xy(clock_tick));
    assert(!tmag5273_async_start_xy(clock_tick + 20));
    assert((cr2 & I2C_CR2_RD_WRN) && ((cr2 >> 16) & 0xff) == 3);
    result(0xe1);  // Fresh count 7, ready, no reset or diagnostic failure.
    tmag5273_xy_sample_t sample;
    assert(tmag5273_async_take_xy(&sample));
    assert(sample.x == 0x1200 && sample.y == -256 && sample.z == 0);
    assert(sample.sample_tick == 65500u);
    assert(tmag_stats.sample_count == 1);
    assert(!tmag5273_async_take_xy(&sample));
    assert(async_state == ASYNC_IDLE); clock_tick += 100;
    assert(tmag5273_async_start_xy(clock_tick)); result(0xe1);
    assert(!tmag5273_async_take_xy(&sample)); // Duplicate count.
    assert(async_state == ASYNC_IDLE); clock_tick += 100;
    assert(tmag5273_async_start_xy(clock_tick)); result(1);
    assert(tmag5273_async_take_xy(&sample)); // Count 7 -> 0 wrap is fresh.
    uint8_t invalid[] = {0x20, 0x31, 0x23}; // Not ready, reset, diagnostic failure.
    for (unsigned i = 0; i < sizeof(invalid); ++i) {
        assert(async_state == ASYNC_IDLE); clock_tick += 100;
        assert(tmag5273_async_start_xy(clock_tick)); result(invalid[i]);
        assert(!tmag5273_async_take_xy(&sample));
    }
    assert(async_state == ASYNC_IDLE); clock_tick += 100;
    assert(tmag5273_async_start_xy(clock_tick));
    hw_isr = I2C_ISR_NACKF; tmag5273_i2c1_isr();
    assert(tmag_stats.nack_count == 1 && tmag_stats.recover_count == 1);
    assert(async_state == ASYNC_IDLE);
    assert(tmag5273_async_start_xy(clock_tick));
    tmag5273_async_cancel(); assert(async_state == ASYNC_IDLE && !async_sample_ready);
    return 0;
}
