#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/systick.h>
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * TMAG5273 Hall-Effect Sensor Driver
 * Datasheet: Texas Instruments TMAG5273
 * ============================================================================ */

/* I2C Addresses for different TMAG5273 variants (7-bit) */
#define TMAG5273A_I2C_ADDR    0x35
#define TMAG5273B_I2C_ADDR    0x22
#define TMAG5273C_I2C_ADDR    0x78
#define TMAG5273D_I2C_ADDR    0x44

/* Select your variant here */
#define TMAG5273_I2C_ADDR     TMAG5273A_I2C_ADDR

/* Register Addresses */
#define TMAG5273_DEVICE_CONFIG_1     0x00
#define TMAG5273_DEVICE_CONFIG_2     0x01
#define TMAG5273_SENSOR_CONFIG_1     0x02
#define TMAG5273_SENSOR_CONFIG_2     0x03
#define TMAG5273_X_THR_CONFIG        0x04
#define TMAG5273_Y_THR_CONFIG        0x05
#define TMAG5273_Z_THR_CONFIG        0x06
#define TMAG5273_T_CONFIG            0x07
#define TMAG5273_INT_CONFIG_1        0x08
#define TMAG5273_MAG_GAIN_CONFIG     0x09
#define TMAG5273_MAG_OFFSET_CONFIG_1 0x0A
#define TMAG5273_MAG_OFFSET_CONFIG_2 0x0B
#define TMAG5273_I2C_ADDRESS         0x0C
#define TMAG5273_DEVICE_ID           0x0D
#define TMAG5273_MANUFACTURER_ID_LSB 0x0E
#define TMAG5273_MANUFACTURER_ID_MSB 0x0F
#define TMAG5273_T_MSB_RESULT        0x10
#define TMAG5273_T_LSB_RESULT        0x11
#define TMAG5273_X_MSB_RESULT        0x12
#define TMAG5273_X_LSB_RESULT        0x13
#define TMAG5273_Y_MSB_RESULT        0x14
#define TMAG5273_Y_LSB_RESULT        0x15
#define TMAG5273_Z_MSB_RESULT        0x16
#define TMAG5273_Z_LSB_RESULT        0x17
#define TMAG5273_CONV_STATUS         0x18
#define TMAG5273_ANGLE_MSB_RESULT    0x19
#define TMAG5273_ANGLE_LSB_RESULT    0x1A
#define TMAG5273_MAGNITUDE_RESULT    0x1B

/* DEVICE_CONFIG_1 Register Bits */
#define TMAG5273_CRC_EN              (1 << 7)  /* CRC enable */
#define TMAG5273_MAG_TEMPCO_SHIFT    5         /* Magnet temp coefficient */
#define TMAG5273_CONV_AVG_SHIFT      2         /* Conversion averaging */
#define TMAG5273_READ_MODE           (1 << 1)  /* 0=Standard, 1=1-byte special read */

/* DEVICE_CONFIG_2 Register Bits */
#define TMAG5273_THR_HYST_SHIFT      5
#define TMAG5273_LP_LN               (1 << 4)  /* Low power/noise mode */
#define TMAG5273_I2C_GLITCH_FILTER   (1 << 3)
#define TMAG5273_TRIGGER_MODE        (1 << 2)  /* INT pin trigger mode */
#define TMAG5273_OPERATING_MODE_SHIFT 0        /* 00=Standby, 01=Sleep, 10=Continuous, 11=WakeupSleep */

/* SENSOR_CONFIG_1 Register Bits */
#define TMAG5273_MAG_CH_EN_SHIFT     4         /* Magnetic channel enable */
#define TMAG5273_SLEEPTIME_SHIFT     0

/* MAG_CH_EN Values */
#define TMAG5273_MAG_CH_OFF          0x0
#define TMAG5273_MAG_CH_X            0x1
#define TMAG5273_MAG_CH_Y            0x2
#define TMAG5273_MAG_CH_Z            0x3
#define TMAG5273_MAG_CH_XY           0x4
#define TMAG5273_MAG_CH_XZ           0x5
#define TMAG5273_MAG_CH_YZ           0x6
#define TMAG5273_MAG_CH_XYZ          0x7
#define TMAG5273_MAG_CH_XYZT         0x8
#define TMAG5273_MAG_CH_XYT          0x9
#define TMAG5273_MAG_CH_XZT          0xA
#define TMAG5273_MAG_CH_YZT          0xB
#define TMAG5273_MAG_CH_XT           0xC
#define TMAG5273_MAG_CH_YT           0xD
#define TMAG5273_MAG_CH_ZT           0xE
#define TMAG5273_MAG_CH_T            0xF

/* SENSOR_CONFIG_2 Register Bits */
#define TMAG5273_THRX_COUNT          (1 << 6)
#define TMAG5273_MAG_THR_DIR         (1 << 5)
#define TMAG5273_MAG_GAIN_CH_SHIFT   4
#define TMAG5273_ANGLE_EN_SHIFT      2         /* Angle calculation axis pair */
#define TMAG5273_X_Y_RANGE_SHIFT     1
#define TMAG5273_Z_RANGE             (1 << 0)

/* ANGLE_EN Values - Which axes to use for angle calculation */
#define TMAG5273_ANGLE_OFF           0x0  /* No angle calculation */
#define TMAG5273_ANGLE_XY            0x1  /* X-Y plane angle */
#define TMAG5273_ANGLE_YZ            0x2  /* Y-Z plane angle */
#define TMAG5273_ANGLE_XZ            0x3  /* X-Z plane angle */

/* CONV_AVG Values - Averaging options */
#define TMAG5273_CONV_AVG_1X         0x0  /* No averaging (fastest) */
#define TMAG5273_CONV_AVG_2X         0x1
#define TMAG5273_CONV_AVG_4X         0x2
#define TMAG5273_CONV_AVG_8X         0x3
#define TMAG5273_CONV_AVG_16X        0x4
#define TMAG5273_CONV_AVG_32X        0x5

/* Operating Modes */
#define TMAG5273_OP_STANDBY          0x0
#define TMAG5273_OP_SLEEP            0x1
#define TMAG5273_OP_CONTINUOUS       0x2
#define TMAG5273_OP_WAKEUP_SLEEP     0x3

/* ============================================================================
 * GPIO Definitions for Motor Control
 * ============================================================================ */
#define HALLA_PORT   GPIOC
#define HALLA_PIN    GPIO14

#define HALLB_PORT   GPIOC
#define HALLB_PIN    GPIO15

#define HALLC_PORT   GPIOA
#define HALLC_PIN    GPIO0

#define BRAKE_PORT   GPIOA
#define BRAKE_PIN    GPIO1

/**
 * Manual Clock Setup for STM32L0
 * Goal: 32MHz System Clock
 * Source: HSI16 (16MHz Internal)
 * PLL: Mul x4, Div /2 => (16 * 4) / 2 = 32MHz
 */
const struct rcc_clock_scale rcc_hsi16_32mhz = {
    .pll_source = RCC_CFGR_PLLSRC_HSI16_CLK,
    .pll_mul = RCC_CFGR_PLLMUL_MUL4,
    .pll_div = RCC_CFGR_PLLDIV_DIV2,
    .hpre = RCC_CFGR_HPRE_NODIV,
    .ppre1 = RCC_CFGR_PPRE1_NODIV,
    .ppre2 = RCC_CFGR_PPRE2_NODIV,
    .voltage_scale = PWR_SCALE1,
    .flash_waitstates = 1,
    .ahb_frequency = 32000000,
    .apb1_frequency = 32000000,
    .apb2_frequency = 32000000,
};

static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hsi16_32mhz);

    // Enable Peripheral Clocks
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_TIM2);
}

// --- SysTick & Delay Helper ---
volatile uint32_t system_millis = 0;

void sys_tick_handler(void);
void sys_tick_handler(void)
{
    system_millis++;
}

static void systick_setup(void)
{
    // Clock = AHB = 32MHz
    // Tick = 1ms => Reload = 32,000,000 / 1000 - 1 = 31999
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(31999);
    systick_interrupt_enable();
    systick_counter_enable();
    
    // Ensure global interrupts are enabled (CM0/CM3)
    __asm__("cpsie i"); 
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = system_millis;
    while ((system_millis - start) < ms);
}

static void usart2_setup(void)
{
    // MSP: PA9 (TX), PA10 (RX) -> AF4
    gpio_set_af(GPIOA, GPIO_AF4, GPIO9 | GPIO10);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);

    // Config: 921600 baud, 8N1
    usart_set_baudrate(USART2, 921600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable(USART2);
}

static void spi1_setup(void)
{
    // MSP: PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI) -> AF0
    gpio_set_af(GPIOA, GPIO_AF0, GPIO4 | GPIO5 | GPIO6 | GPIO7);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO4 | GPIO5 | GPIO6 | GPIO7);

    // Reset SPI1 to clear any state
    rcc_periph_reset_pulse(RST_SPI1);

    // Master, CPOL=0, CPHA=1 (2nd Edge - Falling Edge sampling)
    // 16-bit Data Frame, MSB First
    spi_init_master(SPI1, 
                    SPI_CR1_BAUDRATE_FPCLK_DIV_8, 
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, 
                    SPI_CR1_CPHA_CLK_TRANSITION_2, 
                    SPI_CR1_DFF_16BIT,             
                    SPI_CR1_MSBFIRST);

    spi_enable_ss_output(SPI1); // Hardware NSS
    spi_enable(SPI1);
}

static void i2c1_setup(void)
{
    // MSP: PB6 (SCL), PB7 (SDA) -> AF1
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, GPIO6 | GPIO7);

    i2c_peripheral_disable(I2C1);

    /* Fast Mode Plus timing for ~1MHz I2C (TMAG5273 supports up to 1MHz)
     * With 32MHz I2CCLK:
     * PRESC=0, SCLDEL=1, SDADEL=0, SCLH=4, SCLL=9
     * This gives approximately 1MHz SCL with proper setup/hold times
     * 
     * For Fast Mode (400kHz), use: 0x00100413 (original CubeMX value)
     * For Fast Mode Plus (~1MHz): 0x00100109 
     */
    // I2C_TIMINGR(I2C1) = 0x00100109;  /* ~1MHz for fastest TMAG5273 reads */
    I2C_TIMINGR(I2C1) = 0x00100413;  

    i2c_peripheral_enable(I2C1);
}

/* ============================================================================
 * TMAG5273 Driver Functions
 * ============================================================================ */

/**
 * Write a single register to TMAG5273
 */
static void tmag5273_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    i2c_transfer7(I2C1, TMAG5273_I2C_ADDR, data, 2, NULL, 0);
}

/**
 * Read a single register from TMAG5273
 */
static uint8_t tmag5273_read_reg(uint8_t reg)
{
    uint8_t value = 42;
    i2c_transfer7(I2C1, TMAG5273_I2C_ADDR, &reg, 1, &value, 1);
    return value;
}

/**
 * Read multiple consecutive registers (burst read)
 * This is faster than individual reads for multiple registers
 */
static void tmag5273_read_regs(uint8_t start_reg, uint8_t *data, uint8_t len)
{
    i2c_transfer7(I2C1, TMAG5273_I2C_ADDR, &start_reg, 1, data, len);
}

/**
 * Initialize TMAG5273 for fastest angle reading using X-Z axes
 * 
 * Configuration for maximum speed:
 * - Continuous measurement mode
 * - No averaging (1x) for fastest updates
 * - Enable X and Z channels for X-Z plane angle calculation
 * - Low noise mode disabled (faster)
 * - CRC disabled (faster I2C)
 */
static bool tmag5273_init_fast_angle(void)
{
    uint8_t device_id;
    
    /* Read Device ID to verify communication */
    device_id = tmag5273_read_reg(TMAG5273_DEVICE_ID);
    
    /* Device ID should be 0x49 for TMAG5273 */
    if ((device_id & 0x3F) == 0) {
        return false;  /* Communication failed */
    }
    
    /* DEVICE_CONFIG_1:
     * - CRC disabled (bit 7 = 0) - faster I2C
     * - MAG_TEMPCO = 0 (no temp compensation)
     * - CONV_AVG = 0 (1x, no averaging) - fastest
     * - READ_MODE = 0 (standard I2C read)
     */
    tmag5273_write_reg(TMAG5273_DEVICE_CONFIG_1, 
        (TMAG5273_CONV_AVG_1X << TMAG5273_CONV_AVG_SHIFT));
    
    /* SENSOR_CONFIG_1:
     * - SLEEPTIME = 0 (1ms, not used in continuous mode)
     * - MAG_CH_EN = XYZ (0x7) - Enable all channels to debug which axes respond
     */
    tmag5273_write_reg(TMAG5273_SENSOR_CONFIG_1,
        (0 << TMAG5273_SLEEPTIME_SHIFT) |
        (TMAG5273_MAG_CH_XYZ << TMAG5273_MAG_CH_EN_SHIFT));
    
    /* SENSOR_CONFIG_2:
     * - ANGLE_EN = XY (0x1) - Calculate angle from X-Y plane
     * - Z_RANGE = 0 (lower range for better sensitivity)
     * - X_Y_RANGE = 0 (±40mT range for A1 variant)
     */
    tmag5273_write_reg(TMAG5273_SENSOR_CONFIG_2,
        (TMAG5273_ANGLE_XY << TMAG5273_ANGLE_EN_SHIFT) |
        (0 << TMAG5273_X_Y_RANGE_SHIFT) |
        (0 << 0));  /* Z_RANGE = 0 */
    
    /* INT_CONFIG_1:
     * - MASK_INTB = 1 (bit 0) - REQUIRED when INT pin is not connected/floating!
     * This masks the interrupt function and prevents floating INT issues
     */
    tmag5273_write_reg(TMAG5273_INT_CONFIG_1, 0x01);  /* MASK_INTB = 1 */
    
    /* DEVICE_CONFIG_2:
     * - LP_LN = 0 (low power mode, but we use continuous anyway)
     * - I2C_GLITCH_FILTER = 0 (disabled for speed)
     * - TRIGGER_MODE = 0 (I2C command trigger)
     * - OPERATING_MODE = 2 (Continuous measurement) - keeps converting
     */
    tmag5273_write_reg(TMAG5273_DEVICE_CONFIG_2,
        (TMAG5273_OP_CONTINUOUS << TMAG5273_OPERATING_MODE_SHIFT));
    
    return true;
}

/**
 * Read angle result (fastest method - direct register read)
 * Returns angle in degrees * 16 (fixed point, divide by 16 for degrees)
 * Range: 0 to 360 degrees
 * 
 * The TMAG5273 CORDIC engine calculates angle from the selected axes.
 * Result is 9-bit (0-511 representing 0-360°)
 * Resolution: 360/512 ≈ 0.703° per LSB
 */
static uint16_t tmag5273_read_angle_raw(void)
{
    uint8_t data[2];
    
    /* Burst read angle MSB and LSB in one transaction (fastest) */
    tmag5273_read_regs(TMAG5273_ANGLE_MSB_RESULT, data, 2);
    
    /* Angle is 9 bits: MSB[0:0] is bit 8, LSB[7:0] is bits 7:0 */
    return ((uint16_t)(data[0] & 0x01) << 8) | data[1];
}

/**
 * Read angle in degrees (floating point)
 * Returns: 0.0 to 360.0 degrees
 */
static float tmag5273_read_angle_deg(void)
{
    uint16_t raw = tmag5273_read_angle_raw();
    /* 9-bit value (0-511) maps to 0-360 degrees */
    return (float)raw * 360.0f / 512.0f;
}

/**
 * Read angle in fixed-point (faster than float)
 * Returns: angle in 1/10 degrees (0-3600 for 0-360°)
 */
static uint16_t tmag5273_read_angle_tenths(void)
{
    uint16_t raw = tmag5273_read_angle_raw();
    /* 9-bit value (0-511) maps to 0-3600 (tenths of degrees) */
    return (uint16_t)(((uint32_t)raw * 3600) / 512);
}

/**
 * Read Z-axis magnetic field (raw 12-bit signed value)
 * For when you just need Z-axis magnitude, not angle
 */
static int16_t tmag5273_read_z_raw(void)
{
    uint8_t data[2];
    
    tmag5273_read_regs(TMAG5273_Z_MSB_RESULT, data, 2);
    
    /* 12-bit signed result, upper 4 bits of MSB are flags */
    int16_t raw = ((int16_t)(data[0] & 0x0F) << 8) | data[1];
    
    /* Sign extend from 12-bit to 16-bit */
    if (raw & 0x0800) {
        raw |= 0xF000;
    }
    
    return raw;
}

/**
 * Fast combined read - get angle and magnitude in one I2C transaction
 * This is the fastest way to get both values
 */
static void tmag5273_read_angle_magnitude(uint16_t *angle_raw, uint8_t *magnitude)
{
    uint8_t data[3];
    
    /* Burst read: ANGLE_MSB, ANGLE_LSB, MAGNITUDE (3 bytes) */
    tmag5273_read_regs(TMAG5273_ANGLE_MSB_RESULT, data, 3);
    
    *angle_raw = ((uint16_t)(data[0] & 0x01) << 8) | data[1];
    *magnitude = data[2];
}

/**
 * Check if new conversion result is available
 */
static bool tmag5273_data_ready(void)
{
    uint8_t status = tmag5273_read_reg(TMAG5273_CONV_STATUS);
    return (status & 0x01) != 0;  /* Bit 0 = CONV_COMPLETE */
}

/**
 * Print a hex byte over UART
 */
static void print_hex(uint8_t val)
{
    const char hex[] = "0123456789ABCDEF";
    usart_send_blocking(USART2, hex[(val >> 4) & 0x0F]);
    usart_send_blocking(USART2, hex[val & 0x0F]);
}

/**
 * Print a string over UART
 */
static void print_str(const char *s)
{
    while (*s) {
        usart_send_blocking(USART2, *s++);
    }
}

/**
 * Print an unsigned integer as decimal
 */
static void print_uint(uint16_t val)
{
    char buf[6];
    int i = 0;
    
    if (val == 0) {
        usart_send_blocking(USART2, '0');
        return;
    }
    
    /* Build digits in reverse */
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    
    /* Print in correct order */
    while (i > 0) {
        usart_send_blocking(USART2, buf[--i]);
    }
}

/**
 * Debug function - dump ALL registers to find where data might be
 */
static void tmag5273_debug_dump(void)
{
    uint8_t i;
    uint8_t data[32];
    
    print_str("=== TMAG5273 Full Register Dump ===\r\n");
    
    /* Read ALL registers 0x00 - 0x1B one by one */
    print_str("Individual reads:\r\n");
    for (i = 0; i <= 0x1B; i++) {
        print_str("Reg 0x");
        print_hex(i);
        print_str(": 0x");
        print_hex(tmag5273_read_reg(i));
        print_str("\r\n");
    }
    
    print_str("\r\n--- Burst read all result regs (0x10-0x1B) ---\r\n");
    /* Try burst read from 0x10 to 0x1B (12 bytes) */
    tmag5273_read_regs(0x10, data, 12);
    for (i = 0; i < 12; i++) {
        print_str("0x");
        print_hex(0x10 + i);
        print_str(": 0x");
        print_hex(data[i]);
        print_str("\r\n");
    }
    
    print_str("\r\n--- Try trigger conversion ---\r\n");
    /* Try setting to standby then back to continuous to trigger fresh conversion */
    tmag5273_write_reg(TMAG5273_DEVICE_CONFIG_2, TMAG5273_OP_STANDBY);
    delay_ms(1);
    tmag5273_write_reg(TMAG5273_DEVICE_CONFIG_2, TMAG5273_OP_CONTINUOUS);
    delay_ms(5);  /* Wait for conversion */
    
    /* Read status after re-trigger */
    print_str("After retrigger - CONV_STATUS: 0x");
    print_hex(tmag5273_read_reg(TMAG5273_CONV_STATUS));
    print_str("\r\n");
    
    /* Read results after trigger */
    print_str("X: 0x");
    print_hex(tmag5273_read_reg(TMAG5273_X_MSB_RESULT));
    print_hex(tmag5273_read_reg(TMAG5273_X_LSB_RESULT));
    print_str("  Z: 0x");
    print_hex(tmag5273_read_reg(TMAG5273_Z_MSB_RESULT));
    print_hex(tmag5273_read_reg(TMAG5273_Z_LSB_RESULT));
    print_str("  ANG: 0x");
    print_hex(tmag5273_read_reg(TMAG5273_ANGLE_MSB_RESULT));
    print_hex(tmag5273_read_reg(TMAG5273_ANGLE_LSB_RESULT));
    print_str("\r\n\r\n");
}

/**
 * Print angle over UART (for debugging)
 */
static void print_angle(uint16_t angle_tenths)
{
    char buf[16];
    int len = 0;
    
    /* Convert angle in tenths to string (e.g., 1234 -> "123.4") */
    uint16_t degrees = angle_tenths / 10;
    uint16_t frac = angle_tenths % 10;
    
    /* Simple integer to string for degrees */
    if (degrees >= 100) {
        buf[len++] = '0' + (degrees / 100);
        degrees %= 100;
        buf[len++] = '0' + (degrees / 10);
        buf[len++] = '0' + (degrees % 10);
    } else if (degrees >= 10) {
        buf[len++] = '0' + (degrees / 10);
        buf[len++] = '0' + (degrees % 10);
    } else {
        buf[len++] = '0' + degrees;
    }
    
    buf[len++] = '.';
    buf[len++] = '0' + frac;
    buf[len++] = '\r';
    buf[len++] = '\n';
    buf[len] = '\0';
    
    for (int i = 0; i < len; i++) {
        usart_send_blocking(USART2, buf[i]);
    }
}

static void tim2_pwm_setup(void)
{
    // MSP: PB1 -> TIM2_CH4 -> AF5
    gpio_set_af(GPIOB, GPIO_AF5, GPIO1);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);

    // Timer Base: 40kHz @ 32MHz Clock
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, 0); 
    timer_set_period(TIM2, 799);  

    // Channel 4 PWM Mode 1
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_value(TIM2, TIM_OC4, 0); // Start at 0% duty
    timer_enable_oc_output(TIM2, TIM_OC4);
    
    timer_enable_oc_preload(TIM2, TIM_OC4);
    timer_enable_counter(TIM2);
}

static void motor_gpio_setup(void)
{
    /* Configure Hall sensor outputs and brake control as push-pull outputs */
    
    /* PC14 - HALLA */
    gpio_mode_setup(HALLA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HALLA_PIN);
    gpio_set_output_options(HALLA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, HALLA_PIN);
    gpio_clear(HALLA_PORT, HALLA_PIN);  /* Start low */
    
    /* PC15 - HALLB */
    gpio_mode_setup(HALLB_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HALLB_PIN);
    gpio_set_output_options(HALLB_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, HALLB_PIN);
    gpio_clear(HALLB_PORT, HALLB_PIN);  /* Start low */
    
    /* PA0 - HALLC */
    gpio_mode_setup(HALLC_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HALLC_PIN);
    gpio_set_output_options(HALLC_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, HALLC_PIN);
    gpio_clear(HALLC_PORT, HALLC_PIN);  /* Start low */
    
    /* PA1 - BRAKE */
    gpio_mode_setup(BRAKE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BRAKE_PIN);
    gpio_set_output_options(BRAKE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, BRAKE_PIN);
    gpio_clear(BRAKE_PORT, BRAKE_PIN);  /* Start with brake off */
}


/* Structure to hold all sensor data */
typedef struct {
    float temp_degc;
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    float angle_deg;
    uint8_t status;
} tmag_data_t;

static bool tmag5273_read_all(tmag_data_t *out) {
    uint8_t raw_data[13]; // Registers 0x10 through 0x1B
    
    // BURST READ: Start at 0x10 (Temp MSB) and read 12 bytes
    tmag5273_read_regs(TMAG5273_T_MSB_RESULT, raw_data, 13);
    
    // Parse Temp (0x10, 0x11)
    int16_t t_raw = (raw_data[0] << 8) | raw_data[1];
    out->temp_degc = 25.0f + ((t_raw - 17500) / 60.0f);
    
    // Parse X (0x12, 0x13)
    out->x_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    
    // Parse Y (0x14, 0x15)
    out->y_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    
    // Parse Z (0x16, 0x17)
    out->z_raw = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    
    // Parse Status (0x18)
    out->status = raw_data[8];
    
    // Parse Angle (0x19, 0x1A)
    uint16_t angle_raw = (raw_data[9] << 8) | raw_data[10]; // 9-bit
    out->angle_deg = (float)angle_raw * 360.0f / 8192.0f;
    
    return true;
}

int main(void)
{
    clock_setup();
    systick_setup();
    usart2_setup();
    spi1_setup();
    i2c1_setup();
    tim2_pwm_setup();
    motor_gpio_setup();
    
    delay_ms(5); // Allow sensor power up

    if (!tmag5273_init_fast_angle()) {
        print_str("Init Failed\r\n");
        while(1);
    }

    // --- IMPORTANT: Clear POR Bit ---
    // Read status, verify POR is set, then write 1 to bit 4 to clear it.
    uint8_t status = tmag5273_read_reg(TMAG5273_CONV_STATUS);
    if (status & 0x10) {
        print_str("POR detected. Clearing...\r\n");
        // Write 1 to Bit 4 (POR) to clear it
        tmag5273_write_reg(TMAG5273_CONV_STATUS, 0x10);
    }
    

    tmag_data_t sensor_data;

    print_str("TMAG5273 Ready\r\n");
    print_str("Rotate magnet to see which axes respond\r\n\r\n");
    
    /* Read and display config to verify */
    uint8_t cfg1 = tmag5273_read_reg(TMAG5273_SENSOR_CONFIG_1);
    uint8_t cfg2 = tmag5273_read_reg(TMAG5273_SENSOR_CONFIG_2);
    print_str("CFG1: 0x");
    print_hex(cfg1);
    print_str(" CFG2: 0x");
    print_hex(cfg2);
    print_str("\r\n\r\n");

    while (1) {
        // Read all sensor data
        tmag5273_read_all(&sensor_data);
        
        /* Show ALL three axes to see which one is responding */
        print_str("X:");
        print_hex((sensor_data.x_raw >> 8) & 0xFF);
        print_hex(sensor_data.x_raw & 0xFF);
        
        print_str(" Y:");
        print_hex((sensor_data.y_raw >> 8) & 0xFF);
        print_hex(sensor_data.y_raw & 0xFF);
        
        print_str(" Z:");
        print_hex((sensor_data.z_raw >> 8) & 0xFF);
        print_hex(sensor_data.z_raw & 0xFF);
        
        /* Print angle */
        print_str("  Angle: ");
        int angle_int = (int)sensor_data.angle_deg;
        int angle_frac = (int)((sensor_data.angle_deg - angle_int) * 10);
        if (angle_frac < 0) angle_frac = -angle_frac;
        
        print_uint(angle_int);
        print_str(".");
        usart_send_blocking(USART2, '0' + angle_frac);
        print_str(" deg");
        
        print_str("\r\n");

        gpio_toggle(HALLA_PORT, HALLA_PIN);
        gpio_toggle(HALLB_PORT, HALLB_PIN);
        gpio_toggle(HALLC_PORT, HALLC_PIN);

        
        delay_ms(100);
    }
}



// int main(void)
// {
//     clock_setup();
//     systick_setup();
//     usart2_setup();
//     spi1_setup();
//     i2c1_setup();
//     tim2_pwm_setup();
    
//     /* Wait for TMAG5273 power-up (1ms typical) */
//     delay_ms(2);
    
//     /* Initialize TMAG5273 for fast angle reading */
//     if (!tmag5273_init_fast_angle()) {
//         /* Sensor initialization failed - blink error */
//         while (1) {
//             usart_send_blocking(USART2, 'E');
//             usart_send_blocking(USART2, 'R');
//             usart_send_blocking(USART2, 'R');
//             usart_send_blocking(USART2, '\r');
//             usart_send_blocking(USART2, '\n');
//             delay_ms(1000);
//         }
//     }
    
//     /* Print startup message */
//     const char *msg = "TMAG5273 Ready\r\n";
//     while (*msg) {
//         usart_send_blocking(USART2, *msg++);
//     }
    
//     /* Wait for first conversion to complete */
//     delay_ms(10);
    
//     /* Debug dump - check all registers and raw data */
//     tmag5273_debug_dump();

//     while (1) {
//         /* Read raw bytes directly and print them */
//         uint8_t status = tmag5273_read_reg(TMAG5273_CONV_STATUS);
//         uint8_t x_msb = tmag5273_read_reg(TMAG5273_X_MSB_RESULT);
//         uint8_t x_lsb = tmag5273_read_reg(TMAG5273_X_LSB_RESULT);
//         uint8_t y_msb = tmag5273_read_reg(TMAG5273_Y_MSB_RESULT);
//         uint8_t y_lsb = tmag5273_read_reg(TMAG5273_Y_LSB_RESULT);
//         uint8_t z_msb = tmag5273_read_reg(TMAG5273_Z_MSB_RESULT);
//         uint8_t z_lsb = tmag5273_read_reg(TMAG5273_Z_LSB_RESULT);
//         uint8_t ang_msb = tmag5273_read_reg(TMAG5273_ANGLE_MSB_RESULT);
//         uint8_t ang_lsb = tmag5273_read_reg(TMAG5273_ANGLE_LSB_RESULT);
//         uint8_t cfg1 = tmag5273_read_reg(TMAG5273_SENSOR_CONFIG_1);
//         uint8_t cfg2 = tmag5273_read_reg(TMAG5273_SENSOR_CONFIG_2);
        
//         print_str("CFG:");
//         print_hex(cfg1);
//         print_hex(cfg2);
//         print_str(" ST:");
//         print_hex(status);
//         print_str(" X:");
//         print_hex(x_msb);
//         print_hex(x_lsb);
//         print_str(" Y:");
//         print_hex(y_msb);
//         print_hex(y_lsb);
//         print_str(" Z:");
//         print_hex(z_msb);
//         print_hex(z_lsb);
//         print_str(" A:");
//         print_hex(ang_msb);
//         print_hex(ang_lsb);
//         print_str("\r\n");
        
//         delay_ms(200);
//     }

//     return 0;
// }