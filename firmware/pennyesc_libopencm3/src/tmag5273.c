#include "tmag5273.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>

#ifndef PNY_DRIVE_BRUSHED
#define PNY_DRIVE_BRUSHED 0
#endif

/* Register Addresses */
#define REG_DEVICE_CONFIG_1     0x00
#define REG_DEVICE_CONFIG_2     0x01
#define REG_SENSOR_CONFIG_1     0x02
#define REG_SENSOR_CONFIG_2     0x03
#define REG_INT_CONFIG_1        0x08
#define REG_DEVICE_ID           0x0D
#define REG_T_MSB_RESULT        0x10
#define REG_CONV_STATUS         0x18
#define REG_DEVICE_STATUS       0x1C
#define DEVICE_STATUS_VCC_UV     0x01u

/* Config bit positions and values */
#define CONV_AVG_SHIFT          2
#define CONV_AVG_1X             0x0
#define CONV_AVG_2X             0x1
#define I2C_RD_STANDARD         0x0
#define I2C_RD_16BIT            0x1
#if PNY_DRIVE_BRUSHED
/* Free-running XYZ, 1-byte read command: 16-bit X, Y, Z and CONV_STATUS. */
#define ASYNC_READ_LEN          7u
#else
/* Triggered XY: the read after the trigger write is a standard register
 * read of X and Y as 16-bit values, without CONV_STATUS. */
#define ASYNC_READ_LEN          4u
#endif
#define SLEEPTIME_SHIFT         0
#define MAG_CH_EN_SHIFT         4
#define MAG_CH_XY               0x3
#define MAG_CH_XYZ              0x7
#define ANGLE_EN_SHIFT          2
#define ANGLE_OFF               0x0
#define X_Y_RANGE_SHIFT         1
#define OPERATING_MODE_SHIFT    0
#define OP_STANDBY              0x0
#define OP_CONTINUOUS           0x2
#define TRIGGER_REG             0x92u   /* conversion trigger bit | X_MSB */

#define I2C_WAIT_LIMIT          40000u
/* No TXIE: the only written byte is preloaded before START. DMA receives
 * the data, so no interrupt per byte can stall the bus behind a sector edge. */
#define ASYNC_IRQS              (I2C_CR1_ERRIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_RXDMAEN)
#define ASYNC_DMA_CHANNEL       DMA_CHANNEL3
#define ASYNC_DMA_REQUEST       6u  /* RM0377: I2C1_RX is request 6 on channel 3 */

typedef enum {
    ASYNC_IDLE = 0,
    ASYNC_TRIGGER,
    ASYNC_READ_DATA,
} async_state_t;

static volatile tmag5273_stats_t tmag_stats;
static volatile async_state_t async_state;
static volatile uint8_t async_rx[ASYNC_READ_LEN];
static volatile tmag5273_xy_sample_t async_sample;
static volatile bool async_sample_ready;
static volatile uint16_t async_start_phase_us;
static volatile uint16_t last_sample_start_us;
#if PNY_DRIVE_BRUSHED
static uint8_t last_set_count = 0xffu;
#else
/* Each read returns the conversion the previous transaction triggered at
 * trigger_phase_us. After a failed or stale transaction that is unknown. */
static volatile uint16_t trigger_phase_us;
static volatile bool trigger_pending;
#endif

static void i2c1_recover(void);

static void async_disable(void)
{
    I2C_CR1(I2C1) &= ~ASYNC_IRQS;
    DMA_CCR(DMA1, ASYNC_DMA_CHANNEL) = 0u;
}

static void async_dma_arm(void)
{
    DMA_CCR(DMA1, ASYNC_DMA_CHANNEL) = 0u;
    DMA_CPAR(DMA1, ASYNC_DMA_CHANNEL) = (uint32_t)&I2C_RXDR(I2C1);
    DMA_CMAR(DMA1, ASYNC_DMA_CHANNEL) = (uint32_t)async_rx;
    DMA_CNDTR(DMA1, ASYNC_DMA_CHANNEL) = ASYNC_READ_LEN;
    DMA_CCR(DMA1, ASYNC_DMA_CHANNEL) = DMA_CCR_MINC | DMA_CCR_EN;
}

static void async_clear_flags(void)
{
    I2C_ICR(I2C1) = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF |
                    I2C_ICR_ARLOCF | I2C_ICR_OVRCF | I2C_ICR_TIMOUTCF;
}

static void i2c_start_transfer(bool read, uint8_t nbytes, bool autoend)
{
    I2C_CR2(I2C1) = ((uint32_t)TMAG5273_I2C_ADDR << I2C_CR2_SADD_7BIT_SHIFT) |
                    ((uint32_t)nbytes << I2C_CR2_NBYTES_SHIFT) |
                    (read ? I2C_CR2_RD_WRN : 0u) |
                    (autoend ? I2C_CR2_AUTOEND : 0u) |
                    I2C_CR2_START;
}

static void async_fail(uint32_t isr)
{
    if ((isr & I2C_ISR_NACKF) != 0u) {
        tmag_stats.nack_count++;
    }
    if ((isr & (I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_TIMEOUT)) != 0u) {
        tmag_stats.timeout_count++;
    }
    async_disable();
    async_state = ASYNC_IDLE;
#if !PNY_DRIVE_BRUSHED
    trigger_pending = false;
#endif
    i2c1_recover();
}

static void i2c1_recover(void)
{
    tmag_stats.recover_count++;
    i2c_send_stop(I2C1);
    i2c_clear_stop(I2C1);
    I2C_ICR(I2C1) = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    i2c_peripheral_disable(I2C1);
    I2C_TIMINGR(I2C1) = TMAG5273_I2C_TIMING;
    i2c_peripheral_enable(I2C1);
}

static bool wait_isr(uint32_t mask)
{
    uint32_t limit = I2C_WAIT_LIMIT;
    while (limit-- > 0u) {
        uint32_t isr = I2C_ISR(I2C1);
        if ((isr & mask) != 0u) {
            return true;
        }
        if ((isr & I2C_ISR_NACKF) != 0u) {
            tmag_stats.nack_count++;
            return false;
        }
    }
    tmag_stats.timeout_count++;
    return false;
}

static bool transfer_regs(uint8_t start_reg, uint8_t *data, uint8_t len, bool trigger)
{
    uint8_t command = trigger ? (uint8_t)(start_reg | 0x80u) : start_reg;

    tmag5273_async_cancel();
    i2c_start_transfer(false, 1u, false);

    if (!wait_isr(I2C_ISR_TXIS)) {
        i2c1_recover();
        return false;
    }
    i2c_send_data(I2C1, command);

    if (!wait_isr(I2C_ISR_TC)) {
        i2c1_recover();
        return false;
    }

    i2c_start_transfer(true, len, true);

    for (uint8_t i = 0; i < len; i++) {
        if (!wait_isr(I2C_ISR_RXNE)) {
            i2c1_recover();
            return false;
        }
        data[i] = i2c_get_data(I2C1);
    }

    if (!wait_isr(I2C_ISR_STOPF)) {
        i2c1_recover();
        return false;
    }
    i2c_clear_stop(I2C1);
    return true;
}

static bool write_reg_checked(uint8_t reg, uint8_t value)
{
    tmag5273_async_cancel();
    i2c_start_transfer(false, 2u, true);

    if (!wait_isr(I2C_ISR_TXIS)) {
        i2c1_recover();
        return false;
    }
    i2c_send_data(I2C1, reg);

    if (!wait_isr(I2C_ISR_TXIS)) {
        i2c1_recover();
        return false;
    }
    i2c_send_data(I2C1, value);

    if (!wait_isr(I2C_ISR_STOPF)) {
        i2c1_recover();
        return false;
    }
    i2c_clear_stop(I2C1);
    return true;
}

#if PNY_DRIVE_BRUSHED
static bool direct_read(uint8_t *data, uint8_t len)
{
    tmag5273_async_cancel();
    i2c_start_transfer(true, len, true);

    for (uint8_t i = 0; i < len; i++) {
        if (!wait_isr(I2C_ISR_RXNE)) {
            i2c1_recover();
            return false;
        }
        data[i] = i2c_get_data(I2C1);
    }

    if (!wait_isr(I2C_ISR_STOPF)) {
        i2c1_recover();
        return false;
    }
    i2c_clear_stop(I2C1);
    return true;
}
#endif

static bool write_mode_regs(uint8_t avg, uint8_t read_mode, uint8_t channels, uint8_t op_mode)
{
    return write_reg_checked(REG_DEVICE_CONFIG_1, (avg << CONV_AVG_SHIFT) | read_mode) &&
           write_reg_checked(REG_SENSOR_CONFIG_1,
               (0u << SLEEPTIME_SHIFT) | (channels << MAG_CH_EN_SHIFT)) &&
           write_reg_checked(REG_SENSOR_CONFIG_2,
               (ANGLE_OFF << ANGLE_EN_SHIFT) | (1u << X_Y_RANGE_SHIFT)) &&
           write_reg_checked(REG_INT_CONFIG_1, 0x01u) &&
           write_reg_checked(REG_DEVICE_CONFIG_2, op_mode << OPERATING_MODE_SHIFT);
}

void tmag5273_write_reg(uint8_t reg, uint8_t value)
{
    (void)write_reg_checked(reg, value);
}

uint8_t tmag5273_read_reg(uint8_t reg)
{
    uint8_t value = 0u;
    (void)transfer_regs(reg, &value, 1u, false);
    return value;
}

void tmag5273_async_setup(void)
{
    dma_set_channel_request(DMA1, ASYNC_DMA_CHANNEL, ASYNC_DMA_REQUEST);
}

bool tmag5273_init(void)
{
    /* The sensor can retain fast-read mode across an MCU reset. */
    if (!write_reg_checked(REG_DEVICE_CONFIG_1, I2C_RD_STANDARD)) {
        return false;
    }
    /* Verify communication by reading device ID. */
    uint8_t device_id = tmag5273_read_reg(REG_DEVICE_ID);
    if ((device_id & 0x3F) == 0) {
        return false;
    }

    /* A supply ramp can latch undervoltage before the MCU starts. Acknowledge
     * it once at initialization; fresh conversions still reject live faults. */
    return tmag5273_set_mode(TMAG5273_MODE_FULL_XYZ) &&
           write_reg_checked(REG_DEVICE_STATUS, DEVICE_STATUS_VCC_UV);
}

bool tmag5273_set_mode(tmag5273_mode_t mode)
{
    if (mode == TMAG5273_MODE_TRIGGER_XY) {
        return write_mode_regs(CONV_AVG_1X, I2C_RD_STANDARD, MAG_CH_XY, OP_STANDBY);
    }
#if PNY_DRIVE_BRUSHED
    if (mode == TMAG5273_MODE_FAST_XYZ) {
        return write_mode_regs(CONV_AVG_1X, I2C_RD_16BIT, MAG_CH_XYZ, OP_CONTINUOUS);
    }
#endif
    return write_mode_regs(CONV_AVG_2X, I2C_RD_STANDARD, MAG_CH_XYZ, OP_CONTINUOUS);
}

void tmag5273_clear_por(void)
{
    uint8_t status = tmag5273_read_reg(REG_CONV_STATUS);
    if (status & 0x10) {
        tmag5273_write_reg(REG_CONV_STATUS, 0x10);
    }
}

bool tmag5273_read_xyt(tmag_data_t *out)
{
    uint8_t raw[6];
    if (!transfer_regs(REG_T_MSB_RESULT, raw, 6, false)) {
        return false;
    }
    
    int16_t t_raw = (raw[0] << 8) | raw[1];
    out->temp_degc = 25.0f + ((t_raw - 17500) / 60.0f);
    out->x_raw = (int16_t)((raw[2] << 8) | raw[3]);
    out->y_raw = (int16_t)((raw[4] << 8) | raw[5]);
    out->z_raw = 0;
    return true;
}

bool tmag5273_read_all(tmag_data_t *out)
{
    uint8_t raw[8];
    
    /* Burst read: T_MSB, T_LSB, X_MSB, X_LSB, Y_MSB, Y_LSB */
    if (!transfer_regs(REG_T_MSB_RESULT, raw, 8, false)) {
        return false;
    }
    
    /* Temperature: 25°C + (raw - 17500) / 60 */
    int16_t t_raw = (raw[0] << 8) | raw[1];
    out->temp_degc = 25.0f + ((t_raw - 17500) / 60.0f);
    
    /* X and Y magnetic field (signed 16-bit) */
    out->x_raw = (int16_t)((raw[2] << 8) | raw[3]);
    out->y_raw = (int16_t)((raw[4] << 8) | raw[5]);
    out->z_raw = (int16_t)((raw[6] << 8) | raw[7]);
    return true;
}

void tmag5273_get_stats(tmag5273_stats_t *out)
{
    out->timeout_count = tmag_stats.timeout_count;
    out->nack_count = tmag_stats.nack_count;
    out->recover_count = tmag_stats.recover_count;
    out->sample_count = tmag_stats.sample_count;
    out->sample_dt_us = tmag_stats.sample_dt_us;
}

void tmag5273_async_cancel(void)
{
    /* Let a transaction in flight finish (about 100 us): aborting mid-read
     * can leave the sensor holding SDA, and the next blocking access stalls. */
    for (uint16_t i = 0; i < 4000u && async_state != ASYNC_IDLE; i++) {
    }
    nvic_disable_irq(NVIC_I2C1_IRQ);
    async_disable();
    async_state = ASYNC_IDLE;
    async_sample_ready = false;
#if PNY_DRIVE_BRUSHED
    last_set_count = 0xffu;
#else
    trigger_pending = false;
#endif
    if ((I2C_ISR(I2C1) & I2C_ISR_BUSY) != 0u) {
        i2c1_recover();
    } else {
        async_clear_flags();
    }
    nvic_enable_irq(NVIC_I2C1_IRQ);
}

bool tmag5273_async_start_xy(uint16_t start_phase_us)
{
    if (async_state != ASYNC_IDLE || (I2C_ISR(I2C1) & I2C_ISR_BUSY) != 0u) {
        return false;
    }

    async_start_phase_us = start_phase_us;
    async_clear_flags();
    I2C_CR1(I2C1) |= ASYNC_IRQS;
#if PNY_DRIVE_BRUSHED
    async_state = ASYNC_READ_DATA;
    async_dma_arm();
    i2c_start_transfer(true, ASYNC_READ_LEN, true);
#else
    /* Preloading the register byte makes the trigger instant a fixed bus
     * time after START, independent of interrupt latency. */
    async_state = ASYNC_TRIGGER;
    I2C_ISR(I2C1) = I2C_ISR_TXE;
    I2C_TXDR(I2C1) = TRIGGER_REG;
    i2c_start_transfer(false, 1u, false);
#endif
    return true;
}

bool tmag5273_async_take_xy(tmag5273_xy_sample_t *out)
{
    if (!async_sample_ready) {
        return false;
    }

    nvic_disable_irq(NVIC_I2C1_IRQ);
    out->x = async_sample.x;
    out->y = async_sample.y;
    out->z = async_sample.z;
    out->start_phase_us = async_sample.start_phase_us;
    out->end_phase_us = async_sample.end_phase_us;
    out->sample_tick = async_sample.sample_tick;
    async_sample_ready = false;
    nvic_enable_irq(NVIC_I2C1_IRQ);
    return true;
}

/* Returns the sample's conversion phase, or false if it is not a fresh,
 * correctly timed result. */
static bool async_parse(int16_t *x, int16_t *y, uint16_t *phase)
{
    *x = (int16_t)(((uint16_t)async_rx[0] << 8) | async_rx[1]);
    *y = (int16_t)(((uint16_t)async_rx[2] << 8) | async_rx[3]);
#if PNY_DRIVE_BRUSHED
    uint8_t status = async_rx[ASYNC_READ_LEN - 1u];
    uint8_t count = status >> 5;
    bool fresh = (status & 0x13u) == 1u && count != last_set_count;
    last_set_count = count;
    async_sample.z = (int16_t)(((uint16_t)async_rx[4] << 8) | async_rx[5]);
    *phase = async_start_phase_us;
    return fresh;
#else
    /* No status byte: a repeat of the last result is stale, and this
     * transaction's trigger was ignored by the still-running conversion. */
    bool valid = trigger_pending;
    *phase = trigger_phase_us;
    trigger_phase_us = async_start_phase_us;
    trigger_pending = true;
    if (*x == async_sample.x && *y == async_sample.y) {
        trigger_pending = false;
        return false;
    }
    return valid;
#endif
}

bool tmag5273_i2c1_isr(void)
{
    uint32_t isr = I2C_ISR(I2C1);

    if ((isr & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_TIMEOUT)) != 0u) {
        async_fail(isr);
        return true;
    }

    if (async_state == ASYNC_TRIGGER && (isr & I2C_ISR_TC) != 0u) {
        /* Register byte acknowledged: the conversion has started. Read the
         * previous one with a repeated start. */
        async_state = ASYNC_READ_DATA;
        async_dma_arm();
        i2c_start_transfer(true, ASYNC_READ_LEN, true);
        return false;
    }

    if ((isr & I2C_ISR_STOPF) == 0u) {
        return false;
    }
    async_clear_flags();
    int16_t x;
    int16_t y;
    uint16_t phase;
    bool complete = async_state == ASYNC_READ_DATA &&
                    DMA_CNDTR(DMA1, ASYNC_DMA_CHANNEL) == 0u;
#if !PNY_DRIVE_BRUSHED
    if (!complete) {
        trigger_pending = false;
    }
#endif
    if (complete && async_parse(&x, &y, &phase)) {
        async_sample.x = x;
        async_sample.y = y;
        async_sample.sample_tick = phase;
        async_sample.start_phase_us = async_start_phase_us;
        async_sample.end_phase_us = (uint16_t)timer_get_counter(TIM21);
        async_sample_ready = true;
        tmag_stats.sample_dt_us = (uint16_t)(phase - last_sample_start_us);
        last_sample_start_us = phase;
        tmag_stats.sample_count++;
    }
    async_disable();
    async_state = ASYNC_IDLE;
    return true;
}

#if PNY_DRIVE_BRUSHED
bool tmag5273_read_fast(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t raw[ASYNC_READ_LEN];

    if (!direct_read(raw, sizeof(raw))) {
        return false;
    }

    *x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    *y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    *z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
    return true;
}
#endif

bool tmag5273_read_z_fast(int16_t *z)
{
    uint8_t raw[2];
    
    /* Burst read starting at X_MSB (register 0x12): X_MSB, X_LSB, Y_MSB, Y_LSB */
    if (!transfer_regs(0x16u, raw, 2, false)) {
        return false;
    }
    
    *z = (int16_t)((raw[0] << 8) | raw[1]);
    return true;
}

bool tmag5273_read_xyz_fast(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t raw[6];
    
    /* Burst read starting at X_MSB (register 0x12): X_MSB, X_LSB, Y_MSB, Y_LSB */
    if (!transfer_regs(0x12u, raw, 6, false)) {
        return false;
    }
    
    *x = (int16_t)((raw[0] << 8) | raw[1]);
    *y = (int16_t)((raw[2] << 8) | raw[3]);
    *z = (int16_t)((raw[4] << 8) | raw[5]);
    return true;
}
