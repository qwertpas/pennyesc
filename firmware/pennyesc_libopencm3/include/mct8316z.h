#ifndef MCT8316Z_H
#define MCT8316Z_H

#include <stdint.h>
#include <stdbool.h>

/* Register Definitions */
#define MCT8316Z_REG_IC_STATUS        0x00
#define MCT8316Z_REG_STATUS1          0x01
#define MCT8316Z_REG_STATUS2          0x02
#define MCT8316Z_REG_CONTROL1         0x03
#define MCT8316Z_REG_CONTROL2A        0x04
#define MCT8316Z_REG_CONTROL3         0x05
#define MCT8316Z_REG_CONTROL4         0x06
#define MCT8316Z_REG_CONTROL5         0x07
#define MCT8316Z_REG_CONTROL6         0x08
#define MCT8316Z_REG_CONTROL7         0x09
#define MCT8316Z_REG_CONTROL8         0x0A
#define MCT8316Z_REG_CONTROL9         0x0B
#define MCT8316Z_REG_CONTROL10        0x0C

/* Control Register Bit Definitions */
/* Control 2A (0x04) */
#define MCT8316Z_PWM_MODE_ASYNC_DIG   (1 << 1)
#define MCT8316Z_CLR_FLT              (1 << 0)

/* Control 4 (0x06) - OCP Settings */
#define MCT8316Z_DRV_OFF              (1 << 7)  /* 1 = Driver disabled */
#define MCT8316Z_OCP_MODE_MASK        0x03      /* Bits 1-0 */
#define MCT8316Z_OCP_MODE_LATCHED     0x00      /* Latched fault */
#define MCT8316Z_OCP_MODE_RETRY       0x01      /* Auto retry */
#define MCT8316Z_OCP_MODE_REPORT      0x02      /* Report only, no action */
#define MCT8316Z_OCP_MODE_DISABLED    0x03      /* Disabled */

/* Control 8 (0x0A) - Motor Lock Settings */
#define MCT8316Z_MTR_LOCK_MODE_MASK   0x03      /* Bits 1-0 */
#define MCT8316Z_MTR_LOCK_LATCHED     0x00      /* Latched fault */
#define MCT8316Z_MTR_LOCK_RETRY       0x01      /* Auto retry */
#define MCT8316Z_MTR_LOCK_REPORT      0x02      /* Report only, no action */
#define MCT8316Z_MTR_LOCK_DISABLED    0x03      /* Disabled */
#define MCT8316Z_MTR_LOCK_TDET_MASK   0x0C      /* Bits 3-2 */
#define MCT8316Z_MTR_LOCK_TDET_300MS  (0 << 2)
#define MCT8316Z_MTR_LOCK_TDET_500MS  (1 << 2)
#define MCT8316Z_MTR_LOCK_TDET_1000MS (2 << 2)
#define MCT8316Z_MTR_LOCK_TDET_5000MS (3 << 2)

/* Control 3 */
#define MCT8316Z_OVP_EN               (1 << 2)

/* Function Prototypes */
void mct8316z_init(void);
uint16_t mct8316z_read_reg(uint8_t addr);
void mct8316z_write_reg(uint8_t addr, uint8_t data);
void mct8316z_print_all_regs(void);
void mct8316z_set_pwm_mode_async_dig(void);
void mct8316z_clear_faults(void);
void mct8316z_disable_protections(void);  /* Disable OCP and motor lock for debugging */

#endif /* MCT8316Z_H */
