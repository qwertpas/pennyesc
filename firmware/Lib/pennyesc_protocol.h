#ifndef PENNYESC_PROTOCOL_H
#define PENNYESC_PROTOCOL_H

#include <stdint.h>

#define PNY_FRAME_START 0xAA
#define PNY_FRAME_MAX_PAYLOAD 64

#define PNY_LUT_BITS 5
#define PNY_SEGMENT_SIZE (1U << PNY_LUT_BITS)
#define PNY_LUT_SIZE (8U * PNY_SEGMENT_SIZE)

#define PNY_CAL_POINTS_PER_SWEEP 36
#define PNY_CAL_SWEEP_COUNT 2
#define PNY_CAL_TOTAL_POINTS (PNY_CAL_POINTS_PER_SWEEP * PNY_CAL_SWEEP_COUNT)

enum {
    PNY_CMD_GET_STATUS = 0x1,
    PNY_CMD_SET_POSITION = 0x2,
    PNY_CMD_SET_DUTY = 0x3,
    PNY_CMD_CAL_START = 0x4,
    PNY_CMD_CAL_STATUS = 0x5,
    PNY_CMD_CAL_READ_POINT = 0x6,
    PNY_CMD_CAL_WRITE_BLOB = 0x7,
    PNY_CMD_CAL_COMMIT = 0x8,
    PNY_CMD_CAL_CLEAR = 0x9,
    PNY_CMD_CAL_INFO = 0xA,
    PNY_CMD_ENTER_BOOT = 0xB,
    PNY_CMD_SET_ADVANCE = 0xC,
};

#define PNY_BOOT_MAGIC 0x424F4F54u

enum {
    PNY_MODE_IDLE = 0,
    PNY_MODE_RUN = 1,
    PNY_MODE_CAL = 2,
};

enum {
    PNY_RESULT_OK = 0,
    PNY_RESULT_BAD_STATE = 1,
    PNY_RESULT_BAD_ARG = 2,
    PNY_RESULT_NOT_CALIBRATED = 3,
    PNY_RESULT_BUSY = 4,
    PNY_RESULT_RANGE = 5,
    PNY_RESULT_FLASH = 6,
    PNY_RESULT_CRC = 7,
};

enum {
    PNY_FLAG_CAL_VALID = 1 << 0,
    PNY_FLAG_BUSY = 1 << 1,
    PNY_FLAG_POSITION_REACHED = 1 << 2,
    PNY_FLAG_FAULT = 1 << 3,
    PNY_FLAG_SENSOR_OK = 1 << 4,
};

enum {
    PNY_FAULT_SENSOR = 1 << 0,
    PNY_FAULT_UNCALIBRATED = 1 << 1,
    PNY_FAULT_FLASH = 1 << 2,
};

typedef struct __attribute__((packed)) {
    uint8_t result;
    uint8_t mode;
    uint8_t flags;
    uint8_t faults;
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t angle_turn16;
    int32_t position_crad;
    int32_t velocity_crads;
    int16_t duty;
} pny_status_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t result;
    uint8_t total_points;
} pny_cal_start_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t result;
    uint8_t active;
    uint8_t next_index;
    uint8_t total_points;
} pny_cal_status_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t result;
    uint8_t index;
    uint8_t step_index;
    uint8_t sweep_dir;
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t xy_radius;
    uint16_t sample_spread;
} pny_cal_point_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t result;
    uint16_t next_offset;
} pny_cal_write_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t result;
    uint8_t valid;
} pny_cal_commit_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t result;
    uint8_t valid;
    uint16_t blob_size;
    uint32_t blob_crc32;
} pny_cal_info_payload_t;

#endif
