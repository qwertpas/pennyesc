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

typedef enum {
    TMAG5273_MODE_FULL_XYZ = 0,
    TMAG5273_MODE_FAST_XY = 1,
} tmag5273_mode_t;

/**
 * Initialize TMAG5273 in full XYZ mode used by idle and calibration paths.
 * @return true on success, false if communication failed
 */
bool tmag5273_init(void);
bool tmag5273_set_mode(tmag5273_mode_t mode);
bool tmag5273_prime_run_mode(void);

/**
 * Clear power-on-reset flag if set
 * Call after init to acknowledge POR status
 */
void tmag5273_clear_por(void);

/**
 * Read X, Y magnetic field and temperature
 * @param out Pointer to structure to fill with sensor data
 */
bool tmag5273_read_xyt(tmag_data_t *out);

/**
 * Fast read: X and Y only (skips temperature, no float math)
 * Standard register read, used outside the run ISR.
 */
bool tmag5273_read_xy_fast(int16_t *x, int16_t *y);
bool tmag5273_read_xy_fast_triggered(int16_t *x, int16_t *y);
bool tmag5273_read_z_fast(int16_t *z);
bool tmag5273_read_xyz_fast(int16_t *x, int16_t *y, int16_t *z);

bool tmag5273_read_all(tmag_data_t *out);


/**
 * Read a single register (for debugging)
 */
uint8_t tmag5273_read_reg(uint8_t reg);

/**
 * Write a single register (for debugging)
 */
void tmag5273_write_reg(uint8_t reg, uint8_t value);

#endif /* TMAG5273_H */
