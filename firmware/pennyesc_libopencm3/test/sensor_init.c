/* A retained fast-read mode must not turn a field sample into a device ID. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "tmag5273.h"
/* DEFINITIONS */
static uint8_t read_mode, device_status, device_id;
static unsigned writes, fail_write;
static bool configured;
static bool write_reg_checked(uint8_t reg, uint8_t value)
{
    if (++writes == fail_write) return false;
    if (reg == REG_DEVICE_CONFIG_1) {
        read_mode = value;
    } else {
        assert(reg == REG_DEVICE_STATUS && configured);
        assert(value == DEVICE_STATUS_VCC_UV);
        device_status &= ~value;
    }
    return true;
}
uint8_t tmag5273_read_reg(uint8_t reg)
{
    assert(reg == REG_DEVICE_ID);
    return read_mode == I2C_RD_STANDARD ? device_id : 0;
}
bool tmag5273_set_mode(tmag5273_mode_t mode)
{
    assert(mode == TMAG5273_MODE_FULL_XYZ && read_mode == I2C_RD_STANDARD);
    configured = true;
    return true;
}
/* FUNCTIONS */
int main(void)
{
    read_mode = 1; device_id = 1; device_status = 0x0f;
    assert(tmag5273_init());
    assert(configured && device_status == 0x0e); // Preserve all other diagnostics.
    writes = 0; fail_write = 1; configured = false;
    assert(!tmag5273_init() && !configured);
    writes = 0; fail_write = 2;
    assert(!tmag5273_init()); // Failed acknowledgement must not report success.
    writes = 0; fail_write = 0; device_id = 0; configured = false;
    assert(!tmag5273_init() && !configured);
    return 0;
}
