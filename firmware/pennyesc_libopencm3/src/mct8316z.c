#include "mct8316z.h"
#include <libopencm3/stm32/spi.h>
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

void mct8316z_init(void)
{
    // Enable SPI1 clock
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_GPIOA); // For pins

    // MSP: PA5(SCK), PA6(MISO), PA7(MOSI) -> AF0
    // PA4 is Software NSS (GPIO Output)
    gpio_set_af(GPIOA, GPIO_AF0, GPIO5 | GPIO6 | GPIO7);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5 | GPIO6 | GPIO7);

    // Reset SPI1
    rcc_periph_reset_pulse(RST_SPI1);

    // Config: Master, CPOL=0, CPHA=1 (Falling Edge), 16-bit, MSB First
    // Baud rate: 5MHz max. Setup was DIV_8.
    // System clock is 32MHz. 32/8 = 4MHz. Safe.
    spi_init_master(SPI1, 
                    SPI_CR1_BAUDRATE_FPCLK_DIV_8, 
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, 
                    SPI_CR1_CPHA_CLK_TRANSITION_2, 
                    SPI_CR1_DFF_16BIT,             
                    SPI_CR1_MSBFIRST);

    // Use Software Slave Management (SSM=1, SSI=1)
    // This prevents the Master from detecting a Mode Fault when we pull NSS low
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);

    // Configure NSS (PA4) as GPIO Output
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
    gpio_set(GPIOA, GPIO4); // Set High (Idle)

    spi_enable(SPI1);
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
    
    // Select Slave
    gpio_clear(GPIOA, GPIO4);
    
    // Send and Receive
    spi_send(SPI1, cmd);
    uint16_t val = spi_read(SPI1);
    
    // Deselect Slave
    gpio_set(GPIOA, GPIO4);
    
    return val;
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
    
    // Select Slave
    gpio_clear(GPIOA, GPIO4);
    
    // Send and Receive
    spi_send(SPI1, cmd);
    spi_read(SPI1); // Read dummy byte to clear RX buffer
    
    // Deselect Slave
    gpio_set(GPIOA, GPIO4);
}

void mct8316z_set_pwm_mode_async_dig(void)
{
    // 0. Unlock Registers (just in case)
    // Write 011b (0x3) to CONTROL1 (Reg 0x03) bits 2-0
    // Read-Modify-Write is safer to preserve reserved bits, but we know Reset is 0x00.
    // Let's just write 0x03.
    mct8316z_write_reg(MCT8316Z_REG_CONTROL1, 0x03);
    
    // Delay to ensure unlock processes
    for (volatile int x = 0; x < 1000; x++);

    // 1. Read current CONTROL2A (Reg 0x04)
    uint16_t val = mct8316z_read_reg(MCT8316Z_REG_CONTROL2A);
    uint8_t current_data = (uint8_t)(val & 0xFF);

    // 2. Modify PWM_MODE bits (Bits 2-1)
    // Asynchronous rectification with digital Hall = 1h (01b)
    // Clear bits 2-1
    current_data &= ~(0x06); 
    // Set bit 1
    current_data |= MCT8316Z_PWM_MODE_ASYNC_DIG;
    
    // 3. Write back
    mct8316z_write_reg(MCT8316Z_REG_CONTROL2A, current_data);
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

