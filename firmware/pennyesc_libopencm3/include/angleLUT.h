#ifndef ANGLELUT_H
#define ANGLELUT_H

#include <stdint.h>

/**
 * @file angleLUT.h
 * @brief Fast angle calculation using octant-based lookup table
 * 
 * This module provides a fast angle calculation function that uses a pre-computed
 * lookup table to correct for magnetometer non-orthogonality and amplitude mismatch.
 * The algorithm maps magnetometer (x, y) values to a continuous index using octant
 * and slope information, then looks up the corrected angle.
 */

/**
 * Calculate corrected angle from magnetometer readings
 * 
 * This function uses an octant-based lookup table to correct for magnetometer
 * distortion (non-orthogonality, amplitude mismatch, phase shift).
 * 
 * @param x Magnetometer X-axis reading (signed 16-bit)
 * @param y Magnetometer Y-axis reading (signed 16-bit)
 * @return Corrected angle in radians (0 to 2π)
 * 
 * @note The function uses integer math for performance and is optimized for
 *       embedded systems. The LUT size is 2048 entries (8 octants × 256 segments).
 */
float angleLUT_get_angle(int16_t x, int16_t y);

/**
 * Get the size of the lookup table
 * @return Size of the LUT in entries
 */
uint16_t angleLUT_get_size(void);

#endif /* ANGLELUT_H */

