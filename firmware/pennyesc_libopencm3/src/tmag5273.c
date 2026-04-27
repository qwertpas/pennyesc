#include "tmag5273.h"
#include <libopencm3/stm32/i2c.h>

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

static void i2c1_recover(void)
{
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
            return false;
        }
    }
    return false;
}

static bool transfer_regs(uint8_t start_reg, uint8_t *data, uint8_t len, bool trigger)
{
    uint8_t command = trigger ? (uint8_t)(start_reg | 0x80u) : start_reg;

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
    uint8_t data[2] = {reg, value};
    i2c_transfer7(I2C1, TMAG5273_I2C_ADDR, data, 2, NULL, 0);
}

uint8_t tmag5273_read_reg(uint8_t reg)
{
    uint8_t value;
    i2c_transfer7(I2C1, TMAG5273_I2C_ADDR, &reg, 1, &value, 1);
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

bool tmag5273_prime_run_mode(void)
{
    uint8_t raw[4];
    return transfer_regs(0x12u, raw, sizeof(raw), true);
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

bool tmag5273_read_xy_fast_triggered(int16_t *x, int16_t *y)
{
    uint8_t raw[4];

    if (!transfer_regs(0x12u, raw, 4, true)) {
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
