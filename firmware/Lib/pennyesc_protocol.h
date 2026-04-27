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
    PNY_CMD_EXT = 0x0,
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
    PNY_CMD_SET_QUIET = 0xD,
    PNY_CMD_STEP_SET = 0xE,
    PNY_CMD_STEP_TRANSITION = 0xF,
};

enum {
    PNY_EXT_CAPTURE_START = 0x1,
    PNY_EXT_CAPTURE_STATUS = 0x2,
    PNY_EXT_CAPTURE_READ = 0x3,
    PNY_EXT_SET_OBSERVER = 0x4,
};

enum {
    PNY_OBSERVER_RAW = 0,
    PNY_OBSERVER_LP2 = 1,
    PNY_OBSERVER_LP4 = 2,
    PNY_OBSERVER_LP8 = 3,
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
    uint16_t mct_fault_count;
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
    int16_t duty;
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

typedef struct __attribute__((packed)) {
    uint8_t subcmd;
    int16_t duty;
    int16_t advance_deg;
    uint16_t duration_ms;
    uint16_t sample_hz;
} pny_capture_start_payload_t;

typedef struct __attribute__((packed)) {
    uint16_t angle_turn16;
    int16_t rpm;
} pny_capture_sample_t;

typedef struct __attribute__((packed)) {
    uint8_t subcmd;
    uint8_t result;
    uint8_t active;
    uint8_t done;
    uint16_t sample_hz;
    uint16_t duration_ms;
    uint16_t elapsed_ms;
    uint16_t sample_count;
    uint16_t missed_count;
    int16_t start_rpm;
    int16_t last_rpm;
    int16_t peak_rpm;
    int32_t accel_rpm_s;
    uint16_t mct_fault_count;
} pny_capture_status_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t subcmd;
    uint8_t result;
    uint16_t offset;
    uint8_t count;
    pny_capture_sample_t samples[14];
} pny_capture_read_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t subcmd;
    int16_t lead_us;
    uint8_t mode;
} pny_observer_payload_t;

#endif
