#include "pennyesc_calibration.h"
#include <libopencm3/stm32/flash.h>
#include <string.h>

#define PENNYESC_FLASH_BASE 0x08000000u
#define PENNYESC_FLASH_SIZE 16384u
#define PENNYESC_FLASH_PAGE_BYTES 128u
#define PENNYESC_HALF_PAGE_BYTES (FLASH_HALF_PAGE_SIZE * sizeof(uint32_t))
#define PENNYESC_CAL_FLASH_START (PENNYESC_FLASH_BASE + PENNYESC_FLASH_SIZE - PENNYESC_CAL_FLASH_SIZE)
#define PENNYESC_AFFINE_SHIFT 20

typedef struct {
    uint16_t next_offset;
    uint32_t flash_addr;
    uint8_t byte_count;
    bool started;
    uint32_t words[FLASH_HALF_PAGE_SIZE];
} calibration_writer_t;

static const pennyesc_calibration_blob_t *active_blob;
static bool active_valid;
static calibration_writer_t writer;

static void writer_fill_ff(void)
{
    for (uint32_t i = 0; i < FLASH_HALF_PAGE_SIZE; i++) {
        writer.words[i] = 0xFFFFFFFFu;
    }
    writer.byte_count = 0;
}

static bool calibration_blob_valid(const pennyesc_calibration_blob_t *blob)
{
    if (blob->magic != PENNYESC_CAL_MAGIC) {
        return false;
    }
    if (blob->version != PENNYESC_CAL_VERSION) {
        return false;
    }
    if (blob->size != sizeof(*blob)) {
        return false;
    }

    const uint8_t *data = (const uint8_t *)blob;
    uint32_t calc = pennyesc_calibration_crc32(data + 12, blob->size - 12);
    return calc == blob->crc32;
}

static bool writer_flush(void)
{
    uint32_t base_addr = writer.flash_addr;

    __asm__("cpsid i");
    flash_unlock();
    FLASH_PECR |= FLASH_PECR_PROG;
    while ((FLASH_SR & FLASH_SR_BSY) == FLASH_SR_BSY) {
    }
    for (uint32_t i = 0; i < FLASH_HALF_PAGE_SIZE; i++) {
        MMIO32(base_addr + (i * sizeof(uint32_t))) = writer.words[i];
        while ((FLASH_SR & FLASH_SR_BSY) == FLASH_SR_BSY) {
        }
    }
    FLASH_PECR &= ~FLASH_PECR_PROG;
    flash_lock();
    __asm__("cpsie i");

    for (uint32_t i = 0; i < FLASH_HALF_PAGE_SIZE; i++) {
        if (MMIO32(base_addr + (i * sizeof(uint32_t))) != writer.words[i]) {
            return false;
        }
    }

    writer.flash_addr = base_addr + PENNYESC_HALF_PAGE_BYTES;
    writer_fill_ff();
    return true;
}

const pennyesc_calibration_blob_t *pennyesc_calibration_active(void)
{
    return active_valid ? active_blob : 0;
}

bool pennyesc_calibration_valid(void)
{
    return active_valid;
}

uint32_t pennyesc_calibration_crc32(const void *data, uint32_t len)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;

    for (uint32_t i = 0; i < len; i++) {
        crc ^= bytes[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }

    return ~crc;
}

bool pennyesc_calibration_load(void)
{
    const pennyesc_calibration_blob_t *blob =
        (const pennyesc_calibration_blob_t *)(uintptr_t)PENNYESC_CAL_FLASH_START;

    active_valid = calibration_blob_valid(blob);
    active_blob = active_valid ? blob : 0;
    return active_valid;
}

uint8_t pennyesc_calibration_pseudo_index(int32_t x, int32_t y)
{
    uint32_t ax = (x < 0) ? (uint32_t)(-x) : (uint32_t)x;
    uint32_t ay = (y < 0) ? (uint32_t)(-y) : (uint32_t)y;
    uint8_t octant;
    uint32_t ratio;

    if (ax > ay) {
        if (x >= 0) {
            octant = (y >= 0) ? 0u : 7u;
        } else {
            octant = (y >= 0) ? 3u : 4u;
        }
        ratio = ax ? ((ay * PNY_SEGMENT_SIZE) / ax) : 0u;
    } else {
        if (y >= 0) {
            octant = (x >= 0) ? 1u : 2u;
        } else {
            octant = (x >= 0) ? 6u : 5u;
        }
        ratio = ay ? ((ax * PNY_SEGMENT_SIZE) / ay) : 0u;
    }

    if (ratio >= PNY_SEGMENT_SIZE) {
        ratio = PNY_SEGMENT_SIZE - 1u;
    }

    if (octant & 1u) {
        ratio = (PNY_SEGMENT_SIZE - 1u) - ratio;
    }

    return (uint8_t)((octant << PNY_LUT_BITS) | ratio);
}

uint16_t pennyesc_calibration_angle_turn16(int16_t x, int16_t y)
{
    if (!active_valid) {
        return 0;
    }

    const int32_t *a = active_blob->affine_q20;
    int32_t u = (int32_t)(((int64_t)x * a[0] + (int64_t)y * a[1] + a[2]) >> PENNYESC_AFFINE_SHIFT);
    int32_t v = (int32_t)(((int64_t)x * a[3] + (int64_t)y * a[4] + a[5]) >> PENNYESC_AFFINE_SHIFT);
    uint8_t idx = pennyesc_calibration_pseudo_index(u, v);

    return active_blob->angle_lut[idx];
}

void pennyesc_calibration_writer_reset(void)
{
    writer.started = false;
    writer.next_offset = 0;
    writer.flash_addr = PENNYESC_CAL_FLASH_START;
    writer_fill_ff();
}

bool pennyesc_calibration_clear_flash(void)
{
    pennyesc_calibration_writer_reset();

    __asm__("cpsid i");
    flash_unlock();
    for (uint32_t addr = PENNYESC_CAL_FLASH_START;
         addr < PENNYESC_CAL_FLASH_START + PENNYESC_CAL_FLASH_SIZE;
         addr += PENNYESC_FLASH_PAGE_BYTES) {
        flash_erase_page(addr);
    }
    flash_lock();
    __asm__("cpsie i");

    active_valid = false;
    active_blob = 0;
    return true;
}

bool pennyesc_calibration_write_chunk(uint16_t offset, const uint8_t *data, uint8_t len)
{
    if (offset == 0u) {
        if (!pennyesc_calibration_clear_flash()) {
            return false;
        }
        writer.started = true;
    }

    if (!writer.started || offset != writer.next_offset) {
        return false;
    }
    if ((uint32_t)offset + len > sizeof(pennyesc_calibration_blob_t)) {
        return false;
    }

    for (uint8_t i = 0; i < len; i++) {
        ((uint8_t *)writer.words)[writer.byte_count++] = data[i];
        writer.next_offset++;
        if (writer.byte_count == PENNYESC_HALF_PAGE_BYTES) {
            if (!writer_flush()) {
                return false;
            }
        }
    }

    return true;
}

bool pennyesc_calibration_commit(void)
{
    if (!writer.started || writer.next_offset != sizeof(pennyesc_calibration_blob_t)) {
        return false;
    }

    if (writer.byte_count != 0u) {
        if (!writer_flush()) {
            return false;
        }
    }

    writer.started = false;
    return pennyesc_calibration_load();
}
