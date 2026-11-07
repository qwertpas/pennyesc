#ifndef TMAG5273_H
#define TMAG5273_H

#include <stdint.h>
#include <stdbool.h>

/* I2C Address (7-bit) - TMAG5273A variant */
#define TMAG5273_I2C_ADDR    0x35
/* 32 MHz I2C kernel clock. SCLL 0.5 us, SCLH 0.25 us plus rise and filter
 * delay: about 1 MHz, the TMAG5273 FM+ limit (tLOW 0.5 us, tHIGH 0.26 us).
 * The old 0x00600B13 ran near 0.6 MHz, too slow for a trigger-and-read
 * transaction to finish before the conversion it started. */
#ifndef TMAG5273_I2C_TIMING
#define TMAG5273_I2C_TIMING 0x0020070Fu
#endif
/* Sensor data structure */
typedef struct {
    float temp_degc;
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw; 
} tmag_data_t;

typedef enum {
    TMAG5273_MODE_FULL_XYZ = 0,
    /* Brushed: free-running XYZ, read with the 1-byte command. */
    TMAG5273_MODE_FAST_XYZ = 2,
    /* BLDC: standby; each async transaction triggers the next XY conversion. */
    TMAG5273_MODE_TRIGGER_XY = 3,
} tmag5273_mode_t;

typedef struct {
    uint32_t timeout_count;
    uint32_t nack_count;
    uint32_t recover_count;
    uint32_t sample_count;
    uint16_t sample_dt_us;
} tmag5273_stats_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t start_phase_us;
    uint16_t end_phase_us;
    /* Start of the transaction that triggered this conversion (BLDC) or that
     * read it (brushed). The caller adds the sensor delay. */
    uint16_t sample_tick;
} tmag5273_xy_sample_t;

/**
 * Initialize TMAG5273 in full XYZ mode used by idle and calibration paths.
 * @return true on success, false if communication failed
 */
bool tmag5273_init(void);
bool tmag5273_set_mode(tmag5273_mode_t mode);

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

/* Read the channels selected by the current fast run mode. */
bool tmag5273_read_fast(int16_t *x, int16_t *y, int16_t *z);
bool tmag5273_read_z_fast(int16_t *z);
bool tmag5273_read_xyz_fast(int16_t *x, int16_t *y, int16_t *z);

bool tmag5273_read_all(tmag_data_t *out);
void tmag5273_get_stats(tmag5273_stats_t *out);
/* Route the DMA request used by async reads; call once after clocks are on. */
void tmag5273_async_setup(void);
void tmag5273_async_cancel(void);
/* BLDC: write the trigger bit, then read the previous conversion with a
 * repeated start. Brushed: read the free-running result. */
bool tmag5273_async_start_xy(uint16_t start_phase_us);
bool tmag5273_async_take_xy(tmag5273_xy_sample_t *out);
/* Returns true when a transaction finished, successfully or not. */
bool tmag5273_i2c1_isr(void);


/**
 * Read a single register (for debugging)
 */
uint8_t tmag5273_read_reg(uint8_t reg);

/**
 * Write a single register (for debugging)
 */
void tmag5273_write_reg(uint8_t reg, uint8_t value);

#endif /* TMAG5273_H */
