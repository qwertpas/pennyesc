#include "mct8316z.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

/* Helper to calculate parity
 * Returns 1 if 'val' has odd number of 1s, 0 if even.
 * For Even Parity scheme:
 * If data has odd 1s, we add a 1 (P=1) to make total even.
 * If data has even 1s, we add a 0 (P=0) to make total even.
 * So P = parity(data)
 */
static uint8_t calculate_parity(uint16_t val)
{
    // GCC builtin for parity (returns 1 for odd, 0 for even)
    return __builtin_parity(val);
}

static void cs_delay(void)
{
    for (volatile int x = 0; x < 20; x++);
}

static void bit_delay(void)
{
    for (volatile int x = 0; x < 8; x++);
}

static uint16_t mct8316z_transfer(uint16_t cmd)
{
    uint16_t val = 0u;

    gpio_clear(GPIOA, GPIO4);
    bit_delay();

    for (int8_t bit = 15; bit >= 0; bit--) {
        if ((cmd & (1u << bit)) != 0u) {
            gpio_set(GPIOA, GPIO7);
        } else {
            gpio_clear(GPIOA, GPIO7);
        }

        bit_delay();
        gpio_set(GPIOA, GPIO5);
        bit_delay();

        val <<= 1;
        if (gpio_get(GPIOA, GPIO6) != 0u) {
            val |= 1u;
        }

        gpio_clear(GPIOA, GPIO5);
        bit_delay();
    }

    gpio_set(GPIOA, GPIO4);
    cs_delay();

    return val;
}

void mct8316z_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4 | GPIO5 | GPIO7);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO4 | GPIO5 | GPIO7);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO6);

    gpio_set(GPIOA, GPIO4);
    gpio_clear(GPIOA, GPIO5);
    gpio_clear(GPIOA, GPIO7);
}

uint16_t mct8316z_read_reg(uint8_t addr)
{
    // Frame: [R/W (1)] [ADDR (6)] [PARITY (1)] [DATA (8)]
    // Read = 1 at bit 15
    uint16_t cmd = (1 << 15) | ((addr & 0x3F) << 9);
    
    // Calculate parity on the command bits (bits 15-9)
    // We want the final 16-bit word to have Even Parity.
    // Currently cmd has bits 15-9 set, and bits 8-0 are 0.
    if (calculate_parity(cmd)) {
        cmd |= (1 << 8); // Set parity bit if current count is odd
    }
    
    return mct8316z_transfer(cmd);
}

void mct8316z_write_reg(uint8_t addr, uint8_t data)
{
    // Frame: [R/W (1)] [ADDR (6)] [PARITY (1)] [DATA (8)]
    // Write = 0 at bit 15 (so bit 15 is 0)
    // Address is bits 14-9
    uint16_t cmd = ((addr & 0x3F) << 9) | (data & 0xFF);
    
    // Calculate parity on the entire 15 bits (bits 14-0)
    // We want the final 16-bit word to have Even Parity.
    if (calculate_parity(cmd)) {
        cmd |= (1 << 8); // Set parity bit if current count is odd
    }
    
    mct8316z_transfer(cmd);
}

void mct8316z_clear_faults(void)
{
    // CLR_FLT is bit 0 of CONTROL2A (0x04)
    // Writing 1 to this bit clears all latched faults (NPOR, OCP, etc.)
    // This bit is self-clearing (auto-resets to 0)
    
    // Read current CONTROL2A value
    uint16_t val = mct8316z_read_reg(MCT8316Z_REG_CONTROL2A);
    uint8_t current_data = (uint8_t)(val & 0xFF);
    
    // Set CLR_FLT bit
    current_data |= MCT8316Z_CLR_FLT;
    
    // Write back
    mct8316z_write_reg(MCT8316Z_REG_CONTROL2A, current_data);
    
    // Small delay for fault clear to process
    for (volatile int x = 0; x < 1000; x++);
}

static void mct8316z_set_pwm_mode(uint8_t mode_bits)
{
    mct8316z_write_reg(MCT8316Z_REG_CONTROL1, 0x03);
    for (volatile int x = 0; x < 1000; x++);

    mct8316z_write_reg(MCT8316Z_REG_CONTROL2A,
                       MCT8316Z_CONTROL2A_RESERVED |
                       MCT8316Z_SDO_MODE_PUSH_PULL |
                       (mode_bits & 0x06u) |
                       MCT8316Z_CLR_FLT);
    for (volatile int x = 0; x < 1000; x++);

    mct8316z_clear_faults();
}

void mct8316z_set_pwm_mode_async_dig(void)
{
    mct8316z_set_pwm_mode(MCT8316Z_PWM_MODE_ASYNC_DIG);
}

void mct8316z_set_pwm_mode_sync_dig(void)
{
    mct8316z_set_pwm_mode(MCT8316Z_PWM_MODE_SYNC_DIG);
}

void mct8316z_set_hall_hys_high(void)
{
    uint16_t val = mct8316z_read_reg(MCT8316Z_REG_CONTROL7);
    uint8_t data = (uint8_t)(val & 0xFF);

    data |= MCT8316Z_HALL_HYS_HIGH;
    mct8316z_write_reg(MCT8316Z_REG_CONTROL7, data);
}

void mct8316z_set_direction(bool reverse)
{
    uint16_t val = mct8316z_read_reg(MCT8316Z_REG_CONTROL7);
    uint8_t data = (uint8_t)(val & 0xFF);

    if (reverse) {
        data |= MCT8316Z_DIR_REVERSE;
    } else {
        data &= (uint8_t)~MCT8316Z_DIR_REVERSE;
    }
    mct8316z_write_reg(MCT8316Z_REG_CONTROL7, data);
}

void mct8316z_disable_motor_lock(void)
{
    uint16_t val = mct8316z_read_reg(MCT8316Z_REG_CONTROL8);
    uint8_t data = (uint8_t)(val & 0xFF);

    data &= ~(MCT8316Z_MTR_LOCK_MODE_MASK | MCT8316Z_MTR_LOCK_TDET_MASK);
    data |= MCT8316Z_MTR_LOCK_DISABLED;
    mct8316z_write_reg(MCT8316Z_REG_CONTROL8, data);
}

void mct8316z_set_ilim_recir_coast(void)
{
    uint16_t val = mct8316z_read_reg(MCT8316Z_REG_CONTROL5);
    uint8_t data = (uint8_t)(val & 0xFF);

    data |= MCT8316Z_ILIM_RECIR_COAST;
    mct8316z_write_reg(MCT8316Z_REG_CONTROL5, data);
}

// Local print helpers
static void print_str(const char *s)
{
    while (*s) {
        usart_send_blocking(USART2, *s++);
    }
}

static void print_hex(uint16_t val)
{
    const char hex[] = "0123456789ABCDEF";
    usart_send_blocking(USART2, hex[(val >> 12) & 0xF]);
    usart_send_blocking(USART2, hex[(val >> 8) & 0xF]);
    usart_send_blocking(USART2, hex[(val >> 4) & 0xF]);
    usart_send_blocking(USART2, hex[val & 0xF]);
}

void mct8316z_print_all_regs(void)
{
    print_str("\r\n--- MCT8316Z Status Registers ---\r\n");
    
    // Read first 16 registers (Control + Status)
    // Assuming Status are early in the map (0x00 - 0x0A typically)
    for (uint8_t i = 0; i < 0x10; i++) {
        uint16_t val = mct8316z_read_reg(i);
        
        print_str("Reg 0x");
        usart_send_blocking(USART2, "0123456789ABCDEF"[(i >> 4) & 0xF]);
        usart_send_blocking(USART2, "0123456789ABCDEF"[i & 0xF]);
        print_str(": 0x");
        print_hex(val);
        
        // Decode Status bits if possible (bits 15-8 are status in response?)
        // Datasheet says: "response includes 8 bits of status information followed by 8 bits of register data"
        // So MSB byte = Status, LSB byte = Data.
        print_str(" (St: 0x");
        print_hex((val >> 8) & 0xFF); // Just print hex of status byte
        print_str(" Dat: 0x");
        print_hex(val & 0xFF);       // Just print hex of data byte
        print_str(")\r\n");
        
        // Delay slightly
        for (volatile int x = 0; x < 1000; x++);
    }
}

void mct8316z_disable_protections(void)
{
    mct8316z_write_reg(MCT8316Z_REG_CONTROL1, 0x03);
    for (volatile int x = 0; x < 1000; x++);

    mct8316z_write_reg(MCT8316Z_REG_CONTROL3, MCT8316Z_CONTROL3_NO_REPORTS);
    for (volatile int x = 0; x < 1000; x++);

    mct8316z_write_reg(MCT8316Z_REG_CONTROL4, 0x10 | MCT8316Z_OCP_MODE_DISABLED);
    for (volatile int x = 0; x < 1000; x++);

    mct8316z_write_reg(MCT8316Z_REG_CONTROL8,
                       MCT8316Z_MTR_LOCK_RETRY_5000MS |
                       MCT8316Z_MTR_LOCK_TDET_5000MS |
                       MCT8316Z_MTR_LOCK_DISABLED);
    for (volatile int x = 0; x < 1000; x++);

    mct8316z_clear_faults();
}
