#ifndef TMAG5273_H
#define TMAG5273_H

#include <stdint.h>
#include <stdbool.h>

/* I2C Address (7-bit) - TMAG5273A variant */
#define TMAG5273_I2C_ADDR    0x35

/* Sensor data structure */
typedef struct {
    float temp_degc;
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw; 
} tmag_data_t;

/**
 * Initialize TMAG5273 for fast X/Y/Temperature reading
 * Configures: continuous mode, 32x averaging, 80mT range
 * @return true on success, false if communication failed
 */
bool tmag5273_init(void);

/**
 * Clear power-on-reset flag if set
 * Call after init to acknowledge POR status
 */
void tmag5273_clear_por(void);

/**
 * Read X, Y magnetic field and temperature
 * @param out Pointer to structure to fill with sensor data
 */
void tmag5273_read_xyt(tmag_data_t *out);

/**
 * Fast read: X and Y only (skips temperature, no float math)
 * ~30% faster than tmag5273_read_xyt
 */
void tmag5273_read_xy_fast(int16_t *x, int16_t *y);
void tmag5273_read_z_fast(int16_t *z);
void tmag5273_read_xyz_fast(int16_t *x, int16_t *y,int16_t*z);

void tmag5273_read_all(tmag_data_t * out);


/**
 * Read a single register (for debugging)
 */
uint8_t tmag5273_read_reg(uint8_t reg);

/**
 * Write a single register (for debugging)
 */
void tmag5273_write_reg(uint8_t reg, uint8_t value);

#endif /* TMAG5273_H */

