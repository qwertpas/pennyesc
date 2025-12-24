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
#define CONV_AVG_2X             0x1
#define CONV_AVG_32X            0x5
#define SLEEPTIME_SHIFT         0
#define MAG_CH_EN_SHIFT         4
#define MAG_CH_XYT              0x9
#define ANGLE_EN_SHIFT          2
#define ANGLE_OFF               0x0
#define X_Y_RANGE_SHIFT         1
#define OPERATING_MODE_SHIFT    0
#define OP_CONTINUOUS           0x2

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

static void read_regs(uint8_t start_reg, uint8_t *data, uint8_t len)
{
    i2c_transfer7(I2C1, TMAG5273_I2C_ADDR, &start_reg, 1, data, len);
}

bool tmag5273_init(void)
{
    /* Verify communication by reading device ID */
    uint8_t device_id = tmag5273_read_reg(REG_DEVICE_ID);
    if ((device_id & 0x3F) == 0) {
        return false;
    }
    
    /* DEVICE_CONFIG_1: 32x averaging, no CRC */
    tmag5273_write_reg(REG_DEVICE_CONFIG_1, CONV_AVG_2X << CONV_AVG_SHIFT);
    
    /* SENSOR_CONFIG_1: Enable X, Y, Temperature channels */
    tmag5273_write_reg(REG_SENSOR_CONFIG_1,
        (0 << SLEEPTIME_SHIFT) | (MAG_CH_XYT << MAG_CH_EN_SHIFT));
    
    /* SENSOR_CONFIG_2: No angle calc, 80mT range */
    tmag5273_write_reg(REG_SENSOR_CONFIG_2,
        (ANGLE_OFF << ANGLE_EN_SHIFT) | (1 << X_Y_RANGE_SHIFT));
    
    /* INT_CONFIG_1: Mask INT pin (not connected) */
    tmag5273_write_reg(REG_INT_CONFIG_1, 0x01);
    
    /* DEVICE_CONFIG_2: Continuous measurement mode */
    tmag5273_write_reg(REG_DEVICE_CONFIG_2, OP_CONTINUOUS << OPERATING_MODE_SHIFT);
    
    return true;
}

void tmag5273_clear_por(void)
{
    uint8_t status = tmag5273_read_reg(REG_CONV_STATUS);
    if (status & 0x10) {
        tmag5273_write_reg(REG_CONV_STATUS, 0x10);
    }
}

void tmag5273_read_xyt(tmag_data_t *out)
{
    uint8_t raw[6];
    
    /* Burst read: T_MSB, T_LSB, X_MSB, X_LSB, Y_MSB, Y_LSB */
    read_regs(REG_T_MSB_RESULT, raw, 6);
    
    /* Temperature: 25°C + (raw - 17500) / 60 */
    int16_t t_raw = (raw[0] << 8) | raw[1];
    out->temp_degc = 25.0f + ((t_raw - 17500) / 60.0f);
    
    /* X and Y magnetic field (signed 16-bit) */
    out->x_raw = (int16_t)((raw[2] << 8) | raw[3]);
    out->y_raw = (int16_t)((raw[4] << 8) | raw[5]);
}

