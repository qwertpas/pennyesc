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
/* Control 2A */
#define MCT8316Z_PWM_MODE_ASYNC_DIG   (1 << 1)
#define MCT8316Z_CLR_FLT              (1 << 0)

/* Control 3 */
#define MCT8316Z_OVP_EN               (1 << 2)

/* Function Prototypes */
void mct8316z_init(void);
uint16_t mct8316z_read_reg(uint8_t addr);
void mct8316z_write_reg(uint8_t addr, uint8_t data);
void mct8316z_print_all_regs(void);
void mct8316z_set_pwm_mode_async_dig(void);

#endif /* MCT8316Z_H */
