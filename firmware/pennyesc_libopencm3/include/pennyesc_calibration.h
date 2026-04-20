#ifndef PENNYESC_CALIBRATION_H
#define PENNYESC_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>
#include "pennyesc_protocol.h"

#define PENNYESC_CAL_MAGIC 0x314C4143u
#define PENNYESC_CAL_VERSION 1u
#define PENNYESC_CAL_FLASH_SIZE 640u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t crc32;
    int32_t affine_q20[6];
    uint16_t angle_lut[PNY_LUT_SIZE];
    uint16_t fit_max_error_deg_x100;
    uint16_t sweep_delta_deg_x100;
    uint8_t reserved[88];
} pennyesc_calibration_blob_t;

typedef struct __attribute__((packed)) {
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t xy_radius;
    uint16_t sample_spread;
} pennyesc_calibration_point_t;

const pennyesc_calibration_blob_t *pennyesc_calibration_active(void);
bool pennyesc_calibration_load(void);
bool pennyesc_calibration_valid(void);
uint16_t pennyesc_calibration_angle_turn16(int16_t x, int16_t y);
uint8_t pennyesc_calibration_pseudo_index(int32_t x, int32_t y);
uint32_t pennyesc_calibration_crc32(const void *data, uint32_t len);
void pennyesc_calibration_writer_reset(void);
bool pennyesc_calibration_write_chunk(uint16_t offset, const uint8_t *data, uint8_t len);
bool pennyesc_calibration_commit(void);
bool pennyesc_calibration_clear_flash(void);

#endif
