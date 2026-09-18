#ifndef PENNYESC_TIMING_H
#define PENNYESC_TIMING_H

#include <stdbool.h>
#include <stdint.h>

#define COMM_MAX_PERIOD_US 30000u
#define COMM_MIN_PERIOD_US 12u

/* 4295 / 2^32 approximates 1 / 1,000,000 to 8 ppm. No 64-bit divide on M0+. */
static inline int32_t position_delta(int32_t velocity, int32_t us)
{
    return (int32_t)(((int64_t)velocity * (us * 4295)) >> 32);
}

/* All deadlines are less than half a 16-bit timer wrap into the future. */
static inline bool tick_due(uint16_t now, uint16_t deadline)
{
    return (int16_t)(now - deadline) >= 0;
}

static inline uint8_t sector_next(uint8_t sector, int8_t step)
{
    if (step > 0) {
        return sector == 5u ? 0u : (uint8_t)(sector + 1u);
    }
    return sector == 0u ? 5u : (uint8_t)(sector - 1u);
}

/* speed_k is electrical turn16/second divided by 1000. Q8 retains fractions
 * of a microsecond without a division in the commutation interrupt. */
static inline uint32_t sector_period_q8(uint32_t speed_k)
{
    if (speed_k == 0u) {
        return 0u;
    }
    uint32_t period = 2796202667u / speed_k; /* 65536 * 1000 * 256 / 6 */
    return period <= (COMM_MAX_PERIOD_US << 8) ? period : 0u;
}

static inline uint16_t sector_deadline(uint16_t previous, uint32_t period_q8, uint8_t *fraction)
{
    uint32_t total = period_q8 + *fraction;
    *fraction = (uint8_t)total;
    return (uint16_t)(previous + (total >> 8));
}

/* Limit each phase correction to 1/4 sector without changing sector order. */
static inline int16_t sector_correction(uint16_t deadline, uint16_t desired, uint32_t period_q8)
{
    int32_t correction = (int16_t)(desired - deadline);
    int32_t limit = (int32_t)(period_q8 >> 10);
    if (correction > limit) {
        correction = limit;
    } else if (correction < -limit) {
        correction = -limit;
    }
    return (int16_t)correction;
}

#endif
