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
#define PNY_CAPTURE_MAX_SAMPLES 200
#define PNY_CAPTURE_READ_MAX_SAMPLES 14

enum {
    PNY_CMD_GET_STATUS = 0x1,
    PNY_CMD_SET_POSITION = 0x2,
    PNY_CMD_SET_DUTY = 0x3,
    PNY_CMD_CAL = 0x4,
    PNY_CMD_DEBUG = 0x5,
    PNY_CMD_ZERO_POSITION = 0x6,
    PNY_CMD_SET_VELOCITY = 0x7,
    PNY_CMD_SET_CONTROL = 0x8,
    PNY_CMD_BRAKE = 0x9,
    PNY_CMD_SEND_POSITION = 0xA,
    PNY_CMD_ENTER_BOOT = 0xB,
    PNY_CMD_SET_ADVANCE = 0xC,
    PNY_CMD_GET_POS_VEL = 0xE,
    PNY_CMD_SEND_DUTY = 0xF,
};

enum {
    PNY_CAL_START = 0x1,
    PNY_CAL_STATUS = 0x2,
    PNY_CAL_READ_POINT = 0x3,
    PNY_CAL_WRITE_BLOB = 0x4,
    PNY_CAL_COMMIT = 0x5,
    PNY_CAL_CLEAR = 0x6,
    PNY_CAL_INFO = 0x7,
    PNY_CAL_READ_BLOB = 0x8,
};

enum {
    PNY_DEBUG_CAPTURE_START = 0x1,
    PNY_DEBUG_CAPTURE_STATUS = 0x2,
    PNY_DEBUG_CAPTURE_READ = 0x3,
    PNY_DEBUG_SET_OBSERVER = 0x4,
    PNY_DEBUG_STEP_SET = 0x5,
    PNY_DEBUG_STEP_TRANSITION = 0x6,
    PNY_DEBUG_COMM_STATS = 0x7,
};

/* Commutation settings flags (PNY_DEBUG_SET_OBSERVER, 11-byte form). */
enum {
    PNY_COMM_FLAG_GUARD = 0x2,   /* drop field-weakening advance on slip */
    PNY_COMM_FLAG_I2C_FIRST = 0x4, /* sensor I2C interrupt above sector edges */
};

enum {
    PNY_OBSERVER_SAMPLE_LP2 = 0,
    PNY_OBSERVER_SECANT = 1,
    PNY_OBSERVER_SECANT_LP2 = 2,
    PNY_OBSERVER_SECANT_LP4 = 3,
    PNY_OBSERVER_SECANT_LP8 = 4,
    PNY_OBSERVER_SECANT_LP16 = 5,
    PNY_OBSERVER_AB_FAST = 6,
    PNY_OBSERVER_AB_MID = 7,
    PNY_OBSERVER_AB_SLOW = 8,
    PNY_OBSERVER_MAX = PNY_OBSERVER_AB_SLOW,
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

/* Status buffers are word-aligned in RAM; the packed 60-byte wire layout is unchanged. */
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t result;
    uint8_t mode;
    uint8_t flags;
    uint8_t faults;
    int16_t x;
    int16_t y;
    int16_t z;
    uint16_t angle_turn16;
    int32_t position_turn32;
    int32_t velocity_turn32_per_s;
    int16_t duty;
    uint16_t mct_fault_count;
    uint16_t isr_us;
    uint16_t isr_max_us;
    uint16_t i2c_us;
    uint16_t i2c_start_us;
    uint16_t i2c_end_us;
    uint32_t isr_overrun_count;
    uint32_t i2c_timeout_count;
    uint32_t i2c_nack_count;
    uint32_t i2c_recover_count;
    uint32_t uart_overrun_errors;
    uint32_t tmag_sample_count;
    uint16_t tmag_sample_dt_us;
} pny_status_payload_t;

typedef struct __attribute__((packed)) {
    int32_t position_turn32;
    int32_t velocity_turn32_per_s;
} pny_pos_vel_payload_t;

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
    uint16_t commutation_alignment_turn16;
} pny_cal_info_payload_t;

typedef struct __attribute__((packed)) {
    int16_t kp_q8;
    int16_t kd_q8;
    int16_t kv_q8;
    int16_t kf;
    int16_t clip;
} pny_control_payload_t;

typedef struct __attribute__((packed)) {
    int32_t kp_q8;
    int32_t kd_q8;
    int16_t kv_q8;
    int16_t kf;
    int16_t clip;
} pny_brushed_control_payload_t;

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
    uint16_t mct_fault_count;
} pny_capture_status_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t subcmd;
    uint8_t result;
    uint16_t offset;
    uint8_t count;
    pny_capture_sample_t samples[PNY_CAPTURE_READ_MAX_SAMPLES];
} pny_capture_read_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t subcmd;
    int16_t lead_us;
    uint8_t mode;
} pny_observer_payload_t;

/* Commutation tuning. lead_us extrapolates the observer past the sample
 * instant to cover drive delay. fw_us adds field-weakening advance that grows
 * with speed (electrical speed * fw_us), capped at fw_max_deg. Observer gains
 * are shifts: alpha = 2^-alpha_shift, beta and gamma per sensor tick. */
typedef struct __attribute__((packed)) {
    int16_t lead_us;
    uint8_t flags;
    uint8_t alpha_shift;
    uint8_t beta_shift;
    uint8_t gamma_shift;
    int16_t fw_us;
    int16_t fw_max_deg;
} pny_comm_settings_t;

typedef struct __attribute__((packed)) {
    uint8_t subcmd;
    pny_comm_settings_t settings;
} pny_comm_settings_payload_t;

/* Counters since boot unless noted. Innovation is measured minus predicted
 * mechanical angle in turn16 units; innov_mean is an average of |innovation|
 * scaled by 16. Max values reset when read. */
typedef struct __attribute__((packed)) {
    uint32_t samples;
    uint32_t missed_ticks;
    uint16_t guard_events;
    uint16_t comm_faults;
    uint16_t innov_mean_x16;
    uint16_t innov_max;
    int16_t accel_krpm_s;
    int16_t fw_deg;
    uint16_t proc_max_us;
    uint8_t reset_flags;
    uint8_t tick_us;
    uint16_t rejected;
} pny_comm_diag_t;

typedef struct __attribute__((packed)) {
    uint8_t subcmd;
    uint8_t result;
    pny_comm_settings_t settings;
    pny_comm_diag_t diag;
} pny_comm_stats_payload_t;

#endif
