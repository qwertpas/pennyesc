#ifndef PENNYESC_TIMING_H
#define PENNYESC_TIMING_H

#include <stdbool.h>
#include <stdint.h>

#define COMM_MAX_PERIOD_US 30000u
#define COMM_MIN_PERIOD_US 12u

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

#endif
