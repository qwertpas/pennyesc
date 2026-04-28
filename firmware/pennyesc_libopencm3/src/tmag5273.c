#include "tmag5273.h"
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>

/* Register Addresses */
#define REG_DEVICE_CONFIG_1     0x00
#define REG_DEVICE_CONFIG_2     0x01
#define REG_SENSOR_CONFIG_1     0x02
#define REG_SENSOR_CONFIG_2     0x03
#define REG_INT_CONFIG_1        0x08
#define REG_DEVICE_ID           0x0D
#define REG_T_MSB_RESULT        0x10
#define REG_CONV_STATUS         0x18

/* Config bit positions and values */
#define CONV_AVG_SHIFT          2
#define CONV_AVG_1X             0x0
#define CONV_AVG_2X             0x1
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

#define I2C_WAIT_LIMIT          40000u
#define I2C1_TIMING_VALUE       0x00100109u
#define ASYNC_IRQS              (I2C_CR1_ERRIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_TCIE | I2C_CR1_RXIE | I2C_CR1_TXIE)

typedef enum {
    ASYNC_IDLE = 0,
    ASYNC_WRITE_REG,
    ASYNC_READ_DATA,
} async_state_t;

static volatile tmag5273_stats_t tmag_stats;
static volatile async_state_t async_state;
static volatile uint8_t async_rx[4];
static volatile uint8_t async_rx_index;
static volatile tmag5273_xy_sample_t async_sample;
static volatile uint32_t async_sequence;
static volatile bool async_sample_ready;
static volatile uint16_t async_start_phase_us;

static void i2c1_recover(void);

static void async_disable(void)
{
    I2C_CR1(I2C1) &= ~ASYNC_IRQS;
}

static void async_clear_flags(void)
{
    I2C_ICR(I2C1) = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF |
                    I2C_ICR_ARLOCF | I2C_ICR_OVRCF | I2C_ICR_TIMOUTCF;
}

static void async_set_transfer(bool read, uint8_t nbytes, bool autoend)
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
    i2c1_recover();
}

static void i2c1_recover(void)
{
    tmag_stats.recover_count++;
    i2c_send_stop(I2C1);
    i2c_clear_stop(I2C1);
    I2C_ICR(I2C1) = I2C_ICR_STOPCF | I2C_ICR_NACKCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
    i2c_peripheral_disable(I2C1);
    I2C_TIMINGR(I2C1) = I2C1_TIMING_VALUE;
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
    i2c_set_7bit_address(I2C1, TMAG5273_I2C_ADDR);
    i2c_set_write_transfer_dir(I2C1);
    i2c_set_bytes_to_transfer(I2C1, 1u);
    i2c_disable_autoend(I2C1);
    i2c_send_start(I2C1);

    if (!wait_isr(I2C_ISR_TXIS)) {
        i2c1_recover();
        return false;
    }
    i2c_send_data(I2C1, command);

    if (!wait_isr(I2C_ISR_TC)) {
        i2c1_recover();
        return false;
    }

    i2c_set_7bit_address(I2C1, TMAG5273_I2C_ADDR);
    i2c_set_read_transfer_dir(I2C1);
    i2c_set_bytes_to_transfer(I2C1, len);
    i2c_send_start(I2C1);
    i2c_enable_autoend(I2C1);

    for (uint8_t i = 0; i < len; i++) {
        if (!wait_isr(I2C_ISR_RXNE)) {
            i2c1_recover();
            return false;
        }
        data[i] = i2c_get_data(I2C1);
    }

    return true;
}

static bool write_reg_checked(uint8_t reg, uint8_t value)
{
    tmag5273_async_cancel();
    i2c_set_7bit_address(I2C1, TMAG5273_I2C_ADDR);
    i2c_set_write_transfer_dir(I2C1);
    i2c_set_bytes_to_transfer(I2C1, 2u);
    i2c_enable_autoend(I2C1);
    i2c_send_start(I2C1);

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

static void write_mode_regs(uint8_t avg, uint8_t channels, uint8_t op_mode)
{
    tmag5273_write_reg(REG_DEVICE_CONFIG_1, avg << CONV_AVG_SHIFT);
    tmag5273_write_reg(REG_SENSOR_CONFIG_1,
        (0u << SLEEPTIME_SHIFT) | (channels << MAG_CH_EN_SHIFT));
    tmag5273_write_reg(REG_SENSOR_CONFIG_2,
        (ANGLE_OFF << ANGLE_EN_SHIFT) | (1u << X_Y_RANGE_SHIFT));
    tmag5273_write_reg(REG_INT_CONFIG_1, 0x01u);
    tmag5273_write_reg(REG_DEVICE_CONFIG_2, op_mode << OPERATING_MODE_SHIFT);
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

bool tmag5273_init(void)
{
    /* Verify communication by reading device ID */
    uint8_t device_id = tmag5273_read_reg(REG_DEVICE_ID);
    if ((device_id & 0x3F) == 0) {
        return false;
    }

    return tmag5273_set_mode(TMAG5273_MODE_FULL_XYZ);
}

bool tmag5273_set_mode(tmag5273_mode_t mode)
{
    if (mode == TMAG5273_MODE_FAST_XY) {
        write_mode_regs(CONV_AVG_1X, MAG_CH_XY, OP_CONTINUOUS);
    } else {
        write_mode_regs(CONV_AVG_2X, MAG_CH_XYZ, OP_CONTINUOUS);
    }
    return true;
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
}

void tmag5273_async_cancel(void)
{
    async_disable();
    async_state = ASYNC_IDLE;
    async_rx_index = 0;
    async_clear_flags();
}

bool tmag5273_async_start_xy(uint16_t start_phase_us)
{
    if (async_state != ASYNC_IDLE || (I2C_ISR(I2C1) & I2C_ISR_BUSY) != 0u) {
        return false;
    }

    async_start_phase_us = start_phase_us;
    async_rx_index = 0;
    async_state = ASYNC_WRITE_REG;
    async_clear_flags();
    I2C_CR1(I2C1) |= ASYNC_IRQS;
    async_set_transfer(false, 1u, false);
    return true;
}

bool tmag5273_async_take_xy(tmag5273_xy_sample_t *out)
{
    if (!async_sample_ready) {
        return false;
    }

    out->x = async_sample.x;
    out->y = async_sample.y;
    out->start_phase_us = async_sample.start_phase_us;
    out->end_phase_us = async_sample.end_phase_us;
    out->sequence = async_sample.sequence;
    async_sample_ready = false;
    return true;
}

void tmag5273_i2c1_isr(void)
{
    uint32_t isr = I2C_ISR(I2C1);

    if ((isr & (I2C_ISR_NACKF | I2C_ISR_BERR | I2C_ISR_ARLO | I2C_ISR_OVR | I2C_ISR_TIMEOUT)) != 0u) {
        async_fail(isr);
        return;
    }

    if (async_state == ASYNC_WRITE_REG && (isr & I2C_ISR_TXIS) != 0u) {
        I2C_TXDR(I2C1) = 0x12u;
        return;
    }

    if (async_state == ASYNC_WRITE_REG && (isr & I2C_ISR_TC) != 0u) {
        async_state = ASYNC_READ_DATA;
        async_rx_index = 0;
        async_set_transfer(true, 4u, true);
        return;
    }

    if (async_state == ASYNC_READ_DATA && (isr & I2C_ISR_RXNE) != 0u) {
        uint8_t index = async_rx_index;
        if (index < sizeof(async_rx)) {
            async_rx[index] = (uint8_t)I2C_RXDR(I2C1);
            async_rx_index = (uint8_t)(index + 1u);
        } else {
            (void)I2C_RXDR(I2C1);
        }
        isr = I2C_ISR(I2C1);
    }

    if ((isr & I2C_ISR_STOPF) != 0u) {
        async_clear_flags();
        if (async_state == ASYNC_READ_DATA && async_rx_index == sizeof(async_rx)) {
            async_sample.x = (int16_t)((async_rx[0] << 8) | async_rx[1]);
            async_sample.y = (int16_t)((async_rx[2] << 8) | async_rx[3]);
            async_sample.start_phase_us = async_start_phase_us;
            async_sample.end_phase_us = (uint16_t)timer_get_counter(TIM21);
            async_sample.sequence = ++async_sequence;
            async_sample_ready = true;
        }
        async_disable();
        async_state = ASYNC_IDLE;
    }
}

/* Fast read: X and Y only (4 bytes instead of 6, no float math) */
bool tmag5273_read_xy_fast(int16_t *x, int16_t *y)
{
    uint8_t raw[4];
    
    /* Burst read starting at X_MSB (register 0x12): X_MSB, X_LSB, Y_MSB, Y_LSB */
    if (!transfer_regs(0x12u, raw, 4, false)) {
        return false;
    }
    
    *x = (int16_t)((raw[0] << 8) | raw[1]);
    *y = (int16_t)((raw[2] << 8) | raw[3]);
    return true;
}

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
