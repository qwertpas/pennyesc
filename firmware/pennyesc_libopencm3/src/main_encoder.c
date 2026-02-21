#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "mct8316z.h"
#include "tmag5273.h"
#include "angleLUT.h"

#include "esc_address.h"

#include <assert.h>
// ============================================================================ 
//Change this for every ESC you flash, maximum value of 15
#define ESC_ADDRESS ESC1

//WARNING: STATUS AND CMD fields for response and command packets can only be 4 bits wide
#if ESC_ADDRESS > 0xF
#error "Set ESC_ADDRESS to a value that is at most4 bits wide"
#endif 

/* ============================================================================
 * Protocol Constants (inline - no separate header per design principles)
 * ============================================================================ */
#define START_BYTE          0xAA
#define CMD_SET_POSITION    0x01
#define CMD_SET_DUTY        0x02
#define CMD_POLL            0x03

/* Packet sizes */
#define CMD_POLL_LEN        3   /* START + CMD + CRC */
#define CMD_DUTY_LEN        5   /* START + CMD + int16 + CRC */
#define CMD_POSITION_LEN    7   /* START + CMD + int32 + CRC */
#define RESPONSE_LEN        17  /* START + STATUS + int32 + int32 + CRC */

/* Status flags */
#define STATUS_POSITION_REACHED  0x01
#define STATUS_ERROR            0x02

/* Position control */
#define PI_CRAD             314     /* π in centiradians */
#define TWO_PI_CRAD         628     /* 2π in centiradians */
#define POSITION_DEADBAND_CRAD 500    /* ~3 degrees deadband */

/* Timeout */
#define COMM_TIMEOUT_MS     2000

/* Motor parameters */
#define POLE_PAIRS  6

/* Encoder sampling ISR frequency (choose 5000 or 10000) */
#define ISR_FREQ_HZ         5000
#define ISR_PERIOD_US       (1000000 / ISR_FREQ_HZ)  /* 200us at 5kHz, 100us at 10kHz */
#define ISR_TIMER_PERIOD    ((1000000 / ISR_FREQ_HZ) - 1)  /* 199 at 5kHz, 99 at 10kHz */

/* Velocity calculation: update every ~1ms worth of samples */
#define VEL_UPDATE_SAMPLES  (ISR_FREQ_HZ / 1000)  /* 5 at 5kHz, 10 at 10kHz */
#define VEL_MULTIPLIER      1000  /* Convert delta_crad/ms to crad/s */

/* GPIO Definitions for Motor Control */
#define HALLA_PORT   GPIOC
#define HALLA_PIN    GPIO14
#define HALLB_PORT   GPIOC
#define HALLB_PIN    GPIO15
#define HALLC_PORT   GPIOA
#define HALLC_PIN    GPIO0
#define BRAKE_PORT   GPIOA
#define BRAKE_PIN    GPIO1

/* ============================================================================
 * Global State
 * ============================================================================ */

/* Clock configuration: 32MHz from HSI16 via PLL */
static const struct rcc_clock_scale rcc_hsi16_32mhz = {
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

volatile uint32_t system_millis = 0;

/* Multi-turn position tracking (centiradians) */
static volatile int32_t absolute_position_crad = 0;
static volatile int32_t last_angle_crad = 0;
static volatile int32_t velocity_crads = 0;  /* centiradians per second */
static volatile int32_t last_position_crad = 0;

/* Control state */
static volatile bool target_position_set = false;
static volatile int32_t target_position_crad = 0;
static volatile int16_t commanded_duty = 0;
static volatile int16_t current_duty = 0;
static volatile int direction = 0;

/* Communication state */
static volatile uint32_t last_valid_cmd_time = 0;

/* UART RX buffer */
static uint8_t rx_buf[16];
static volatile uint8_t rx_idx = 0;
static volatile uint32_t rx_last_byte_time = 0;
#define RX_INTER_BYTE_TIMEOUT_MS 10

/* DMA circular buffer for UART RX */
#define DMA_RX_BUF_SIZE 32
static volatile uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];
static volatile uint16_t dma_rx_tail = 0;  /* Where we've read up to */

/* Sensor data (shared with ISR) */
static volatile int16_t sensor_x = 0;
static volatile int16_t sensor_y = 0;
static volatile int16_t sensor_z = 0;
static volatile float current_angle_rad = 0;

/* Commutation */
static volatile int advance_deg = 110; //130

/* Debug/benchmark variables (visible in MCUViewer) */
volatile uint32_t isr_duration_us = 0;
volatile uint32_t isr_max_us = 0;
volatile uint32_t isr_count = 0;
volatile uint32_t isr_overrun_count = 0;
volatile uint32_t isr_interval_us = 0;  /* Time between ISR calls (should be ~ISR_PERIOD_US) */
volatile uint32_t isr_interval_min_us = 0xFFFFFFFF;
volatile uint32_t isr_interval_max_us = 0;
volatile uint32_t uart_rx_count = 0;
volatile uint32_t uart_crc_errors = 0;
volatile uint32_t uart_bytes_rx = 0;  /* Raw byte counter for debugging */
volatile uint32_t uart_overrun_errors = 0;  /* ORE error counter */
volatile uint32_t looptime_us = 0;
volatile int32_t debug_position = 0;
volatile int32_t debug_velocity = 0;
volatile int debug_comm_step = 0;

/* Granular ISR timing breakdown (in microseconds) */
volatile uint32_t debug_i2c_us = 0;      /* Time for I2C sensor read */
volatile uint32_t debug_lut_us = 0;      /* Time for angle LUT lookup */
volatile uint32_t debug_update_us = 0;   /* Time for position update + velocity */
volatile uint32_t debug_control_us = 0;  /* Time for control logic + PWM + commutation */

/* ============================================================================
 * System Functions
 * ============================================================================ */

void sys_tick_handler(void)
{
    system_millis++;
}

static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hsi16_32mhz);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_TIM21);
    rcc_periph_clock_enable(RCC_CRC);
}

static void systick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(31999);  /* 1ms at 32MHz */
    systick_interrupt_enable();
    systick_counter_enable();
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = system_millis;
    while ((system_millis - start) < ms);
}

static uint32_t get_time_us(void)
{
    uint32_t millis = system_millis;
    uint32_t ticks_elapsed = 31999 - systick_get_value();
    return (millis * 1000) + (ticks_elapsed >> 5);  /* /32 via shift */
}

/* ============================================================================
 * Hardware CRC-8 (polynomial 0x07)
 * ============================================================================ */

static void crc8_init(void)
{
    /* CRC peripheral already clocked in clock_setup */
    CRC_CR = CRC_CR_RESET;
    
    /* Set 8-bit polynomial size */
    CRC_CR = (CRC_CR & ~CRC_CR_POLYSIZE) | CRC_CR_POLYSIZE_8;
    
    /* Set polynomial to 0x07 (CRC-8/CCITT) */
    CRC_POL = 0x07;
    
    /* Set initial value to 0x00 */
    CRC_INIT = 0x00;
}

static uint8_t crc8_calculate(const uint8_t *data, uint8_t len)
{
    /* Reset CRC to initial value */
    CRC_CR |= CRC_CR_RESET;
    
    /* Feed data byte by byte using 8-bit access */
    for (uint8_t i = 0; i < len; i++) {
        *((volatile uint8_t *)&CRC_DR) = data[i];
    }
    
    return (uint8_t)(CRC_DR & 0xFF);
}

/* ============================================================================
 * UART Setup
 * ============================================================================ */

static void usart2_setup(void)
{
    gpio_set_af(GPIOA, GPIO_AF4, GPIO9 | GPIO10);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD , GPIO_OSPEED_2MHZ , GPIO9 | GPIO10);

    usart_set_baudrate(USART2, 921600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
}

/* ============================================================================
 * DMA Setup for UART RX (circular buffer)
 * ============================================================================ */

static void dma1_ch5_setup(void)
{
    rcc_periph_clock_enable(RCC_DMA);
    dma_channel_reset(DMA1, DMA_CHANNEL5);
    
    /* USART2_RX -> DMA1 Channel 5, request 4 (per RM0377 Table 45) */
    dma_set_channel_request(DMA1, DMA_CHANNEL5, 4);
    
    dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t)&USART_RDR(USART2));
    dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)dma_rx_buf);
    dma_set_number_of_data(DMA1, DMA_CHANNEL5, DMA_RX_BUF_SIZE);
    
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL5);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_HIGH);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL5);
    
    dma_enable_channel(DMA1, DMA_CHANNEL5);
    usart_enable_rx_dma(USART2);
}

/* ============================================================================
 * I2C Setup (1MHz for fast TMAG reads)
 * ============================================================================ */

static void i2c1_setup(void)
{
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, GPIO6 | GPIO7);
    i2c_peripheral_disable(I2C1);
    /* ~1MHz Fast Mode Plus for fast TMAG5273 reads */
    I2C_TIMINGR(I2C1) = 0x00100109;
    i2c_peripheral_enable(I2C1);
}

/* ============================================================================
 * PWM Setup (TIM2 for motor)
 * ============================================================================ */

static void tim2_pwm_setup(void)
{
    gpio_set_af(GPIOB, GPIO_AF5, GPIO1);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, 0);
    // timer_set_period(TIM2, 399);  /* 80kHz PWM */
    timer_set_period(TIM2, 799);  /* 40kHz PWM */
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_value(TIM2, TIM_OC4, 0);
    timer_enable_oc_output(TIM2, TIM_OC4);
    timer_enable_oc_preload(TIM2, TIM_OC4);
    timer_enable_counter(TIM2);
}

/* ============================================================================
 * Timer Setup (TIM21 for encoder sampling at ISR_FREQ_HZ)
 * ============================================================================ */

static void tim21_setup(void)
{
    /* TIM21 is a basic timer on STM32L0 */
    /* 32MHz / 32 = 1MHz timer clock */
    timer_set_prescaler(TIM21, 31);  /* 32MHz / 32 = 1MHz */
    timer_set_period(TIM21, ISR_TIMER_PERIOD);  /* Set by ISR_FREQ_HZ */
    timer_enable_irq(TIM21, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM21_IRQ);
    nvic_set_priority(NVIC_TIM21_IRQ, 0);  /* Highest priority */
    timer_enable_counter(TIM21);
}

/* ============================================================================
 * Motor Control GPIO
 * ============================================================================ */

static void motor_gpio_setup(void)
{
    /* Hall outputs: PC14, PC15, PA0 */
    gpio_mode_setup(HALLA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HALLA_PIN);
    gpio_set_output_options(HALLA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, HALLA_PIN);
    gpio_clear(HALLA_PORT, HALLA_PIN);

    gpio_mode_setup(HALLB_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HALLB_PIN);
    gpio_set_output_options(HALLB_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, HALLB_PIN);
    gpio_clear(HALLB_PORT, HALLB_PIN);

    gpio_mode_setup(HALLC_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HALLC_PIN);
    gpio_set_output_options(HALLC_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, HALLC_PIN);
    gpio_clear(HALLC_PORT, HALLC_PIN);

    /* Brake: PA1 */
    gpio_mode_setup(BRAKE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BRAKE_PIN);
    gpio_set_output_options(BRAKE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, BRAKE_PIN);
    gpio_clear(BRAKE_PORT, BRAKE_PIN);
}

/* Hall pattern lookup: [step] = {A, B, C} */
static const uint8_t hall_pattern[6][3] = {
    {1, 1, 0}, {1, 0, 0}, {1, 0, 1},
    {0, 0, 1}, {0, 1, 1}, {0, 1, 0}
};

static void set_hall_outputs(int step)
{
    step = ((step % 6) + 6) % 6;
    const uint8_t *p = hall_pattern[step];
    if (p[0]) gpio_set(HALLA_PORT, HALLA_PIN); else gpio_clear(HALLA_PORT, HALLA_PIN);
    if (p[1]) gpio_set(HALLB_PORT, HALLB_PIN); else gpio_clear(HALLB_PORT, HALLB_PIN);
    if (p[2]) gpio_set(HALLC_PORT, HALLC_PIN); else gpio_clear(HALLC_PORT, HALLC_PIN);
}

static int angle_to_step(int32_t angle_crad, int advance)
{
    /* Convert centiradians to electrical angle */
    /* 1 crad = 0.57 degrees, so angle_crad * 57.3 / 100 = degrees */
    /* electrical = mechanical * pole_pairs + advance */
    int32_t mech_deg_x10 = (angle_crad * 573) / 100;  /* degrees × 10 */
    int32_t elec_x10 = mech_deg_x10 * POLE_PAIRS + advance * 10;
    
    elec_x10 = elec_x10 % 3600;
    if (elec_x10 < 0) elec_x10 += 3600;
    
    return elec_x10 / 600;
}

/* ============================================================================
 * Multi-Turn Position Tracking
 * ============================================================================ */

static void update_absolute_position(float angle_rad)
{
    int32_t current_crad = (int32_t)(angle_rad * 100.0f);
    int32_t delta = current_crad - last_angle_crad;
    
    /* Handle wraparound (detect >π radian jumps) */
    if (delta > PI_CRAD) delta -= TWO_PI_CRAD;
    if (delta < -PI_CRAD) delta += TWO_PI_CRAD;
    
    absolute_position_crad += delta;
    last_angle_crad = current_crad;
}

/* ============================================================================
 * 10kHz Encoder ISR
 * ============================================================================ */

void tim21_isr(void)
{
    uint32_t t0 = get_time_us();
    timer_clear_flag(TIM21, TIM_SR_UIF);
    
    /* Measure interval between ISR calls (should be ~100us for 10kHz) */
    static uint32_t last_isr_time_us = 0;
    if (last_isr_time_us != 0) {
        uint32_t interval = t0 - last_isr_time_us;
        isr_interval_us = interval;
        if (interval < isr_interval_min_us) isr_interval_min_us = interval;
        if (interval > isr_interval_max_us) isr_interval_max_us = interval;
    }
    last_isr_time_us = t0;
    
    /* ---- SECTION 1: I2C sensor read (fast: X/Y/Z only, no temp) ---- */
    uint32_t t1 = get_time_us();
    int16_t x, y,z;
    //TODO: Remove this blocking read from I2C
    tmag5273_read_xyz_fast(&x, &y, &z);
    sensor_x = x;
    sensor_y = y;
    sensor_z = z;
    
    uint32_t t2 = get_time_us();
    debug_i2c_us = t2 - t1;
    
    /* ---- SECTION 2: LUT lookup ---- */
    float angle_rad = angleLUT_get_angle(sensor_x, sensor_y);
    current_angle_rad = angle_rad;
    uint32_t t3 = get_time_us();
    debug_lut_us = t3 - t2;
    
    /* ---- SECTION 3: Position/velocity update ---- */
    update_absolute_position(angle_rad);
    
    static uint8_t vel_counter = 0;
    vel_counter++;
    if (vel_counter >= VEL_UPDATE_SAMPLES) {  /* Update velocity every ~1ms */
        velocity_crads = (absolute_position_crad - last_position_crad) * VEL_MULTIPLIER;
        last_position_crad = absolute_position_crad;
        vel_counter = 0;
    }
    uint32_t t4 = get_time_us();
    debug_update_us = t4 - t3;
    
    /* ---- SECTION 4: Control logic + PWM + commutation ---- */
    int16_t duty_to_apply = 0;
    if (target_position_set) {
        int32_t error = target_position_crad - absolute_position_crad;
        if (error > POSITION_DEADBAND_CRAD) {
            duty_to_apply = commanded_duty > 0 ? commanded_duty : -commanded_duty;
            direction = 1;
        } else if (error < -POSITION_DEADBAND_CRAD) {
            duty_to_apply = commanded_duty > 0 ? -commanded_duty : commanded_duty;
            direction = -1;
        } else {
            duty_to_apply = 0;
            direction = 0;
        }
    } else {
        duty_to_apply = commanded_duty;
        direction = commanded_duty > 0 ? 1 : (commanded_duty < 0 ? -1 : 0);
    }
    current_duty = duty_to_apply;
    
    int16_t abs_duty = duty_to_apply < 0 ? -duty_to_apply : duty_to_apply;
    if (abs_duty > 799) abs_duty = 799;
    timer_set_oc_value(TIM2, TIM_OC4, abs_duty);
    
    /* Use single-turn angle for commutation (not multi-turn absolute position) */
    int32_t comm_angle_crad = (int32_t)(current_angle_rad * 100.0f);
    if (direction != 0) {
        comm_angle_crad += (velocity_crads / 10000);  /* 0.1ms prediction */
    }
    
    int step = angle_to_step(comm_angle_crad, direction > 0 ? advance_deg : -advance_deg);
    set_hall_outputs(step);
    debug_comm_step = step;
    uint32_t t5 = get_time_us();
    debug_control_us = t5 - t4;
    
    /* Total ISR time */
    uint32_t elapsed = t5 - t0;
    isr_duration_us = elapsed;
    if (elapsed > isr_max_us) isr_max_us = elapsed;
    if (elapsed > ISR_PERIOD_US) isr_overrun_count++;  /* Overrun = took longer than period */
    isr_count++;
    
    /* Update debug vars */
    debug_position = absolute_position_crad;
    debug_velocity = velocity_crads;
}

/* ============================================================================
 * UART Protocol Handler
 * ============================================================================ */

static void send_response(void)
{
    uint8_t tx_buf[RESPONSE_LEN];
    
    /* Build response packet */
    tx_buf[0] = START_BYTE;

    /* Status byte */
    uint8_t status = 0;
    if (target_position_set && direction == 0) {
        status |= STATUS_POSITION_REACHED;
    }

    tx_buf[1] = (ESC_ADDRESS << 4) | status;
    
    /* X reading (int16, little-endian) */
    int16_t x = sensor_x;
    tx_buf[2] = x & 0xFF;
    tx_buf[3] = (x >> 8) & 0xFF;
    
    /* Y reading (int16, little-endian) */
    int16_t y = sensor_y;
    tx_buf[4] = y & 0xFF;
    tx_buf[5] = (y >> 8) & 0xFF;
    
    /* Z reading (int16, little-endian) */
    int16_t z = sensor_z;  /* Not currently tracked in ISR */
    tx_buf[6] = z & 0xFF;
    tx_buf[7] = (z >> 8) & 0xFF;
    
    /* Position (int32, little-endian) */
    int32_t pos = absolute_position_crad;
    tx_buf[8] = pos & 0xFF;
    tx_buf[9] = (pos >> 8) & 0xFF;
    tx_buf[10] = (pos >> 16) & 0xFF;
    tx_buf[11] = (pos >> 24) & 0xFF;
    
    /* Velocity (int32, little-endian) */
    int32_t vel = velocity_crads;
    tx_buf[12] = vel & 0xFF;
    tx_buf[13] = (vel >> 8) & 0xFF;
    tx_buf[14] = (vel >> 16) & 0xFF;
    tx_buf[15] = (vel >> 24) & 0xFF;
    
    /* CRC */
    tx_buf[16] = crc8_calculate(tx_buf, RESPONSE_LEN);
    
    /* Send */
    for (int i = 0; i < RESPONSE_LEN; i++) {
        usart_send_blocking(USART2, tx_buf[i]);
    }
}

static void process_command(void)
{
    uint8_t cmd = rx_buf[1] & 0x0F;
    uint8_t address = rx_buf[1] >> 4; 
    if (address != ESC_ADDRESS) { return; }
    
    switch (cmd) {
        case CMD_POLL:
            /* No payload, just respond */
            break;
            
        case CMD_SET_DUTY: {
            /* 2-byte payload: int16 duty */
            int16_t duty = (int16_t)(rx_buf[2] | (rx_buf[3] << 8));
            commanded_duty = duty;
            target_position_set = false;
            break;
        }
        
        case CMD_SET_POSITION: {
            /* 4-byte payload: int32 position in centiradians */
            int32_t pos = rx_buf[2] | (rx_buf[3] << 8) | 
                         (rx_buf[4] << 16) | (rx_buf[5] << 24);
            target_position_crad = pos;
            target_position_set = true;
            /* Use existing commanded_duty for speed */
            if (commanded_duty == 0) {
                commanded_duty = 150;  /* Default duty if none set */
            }
            break;
        }
        
        default:
            /* Unknown command - ignore */
            return;
    }
    
    /* Update watchdog */
    last_valid_cmd_time = system_millis;
    uart_rx_count++;
    
    /* Send response */
    send_response();
}

static void uart_poll(void)
{
    /* Check for inter-byte timeout (reset incomplete packets) */
    if (rx_idx > 0 && (system_millis - rx_last_byte_time) > RX_INTER_BYTE_TIMEOUT_MS) {
        rx_idx = 0;
    }
    
    /* Calculate head position (where DMA is writing next) */
    uint16_t head = DMA_RX_BUF_SIZE - dma_get_number_of_data(DMA1, DMA_CHANNEL5);
    
    /* Process all available bytes from DMA circular buffer */
    while (dma_rx_tail != head) {
        uint8_t b = dma_rx_buf[dma_rx_tail];
        dma_rx_tail = (dma_rx_tail + 1) % DMA_RX_BUF_SIZE;
        uart_bytes_rx++;  /* Count every byte received */
        rx_last_byte_time = system_millis;
        
        /* Resync: START_BYTE always starts a new packet (handles mid-packet corruption) */
        if (b == START_BYTE) {
            rx_idx = 0;
            rx_buf[rx_idx++] = b;
            continue;
        }
        
        /* Not START_BYTE - only accept if we're mid-packet */
        if (rx_idx == 0) {
            continue;
        }
        
        rx_buf[rx_idx++] = b;
        
        /* Determine expected packet length based on command */
        uint8_t expected_len = 0;
        uint8_t cmd = rx_buf[1] & 0x0F; 
        if (rx_idx >= 2) {
            switch (cmd) {
                case CMD_POLL:
                    expected_len = CMD_POLL_LEN;
                    break;
                case CMD_SET_DUTY:
                    expected_len = CMD_DUTY_LEN;
                    break;
                case CMD_SET_POSITION:
                    expected_len = CMD_POSITION_LEN;
                    break;
                default:
                    /* Unknown command - reset */
                    rx_idx = 0;
                    continue;
            }
        }
        
        /* Check if packet complete */
        if (expected_len > 0 && rx_idx >= expected_len) {
            /* Verify CRC */
            uint8_t crc = crc8_calculate(rx_buf, expected_len - 1);
            if (crc == rx_buf[expected_len - 1]) {
                process_command();
            } else {
                uart_crc_errors++;
            }
            rx_idx = 0;
        }
        
        /* Prevent buffer overflow */
        if (rx_idx >= sizeof(rx_buf)) {
            rx_idx = 0;
        }
    }
}

static void check_timeout(void)
{
    /* Disabled for standalone testing - re-enable for production! */
    #if 0
    if ((system_millis - last_valid_cmd_time) > COMM_TIMEOUT_MS) {
        /* Safety: stop motor if no commands received */
        commanded_duty = 0;
        target_position_set = false;
        timer_set_oc_value(TIM2, TIM_OC4, 0);
    }
    #endif
}

/* ============================================================================
 * Main
 * ============================================================================ */

#define UID_BASE 0x1FF80050


int main(void)
{
    /* Startup delay for flash recovery */
    for (volatile int i = 0; i < 500000; i++);

    uint32_t uid0 = *(uint32_t*)UID_BASE;
    uint32_t uid1 = *(uint32_t*)(UID_BASE + 4);
    uint32_t uid2 = *(uint32_t*)(UID_BASE + 8);
    //(void*)*0x1FF80058 in Watch Window to read uid2 as hex
    //highest byte of uid2 in hexadecimal format is the identifier DD 
    //ex. 0xDD0000 
    //or read uid2 as decimal, then convert to hexadecimal format to read DD


    clock_setup();
    systick_setup();
    crc8_init();
    usart2_setup();
    dma1_ch5_setup();  /* DMA for UART RX circular buffer */
    i2c1_setup();
    tim2_pwm_setup();
    motor_gpio_setup();
    mct8316z_init();

    delay_ms(5);

    mct8316z_set_pwm_mode_async_dig();
    mct8316z_write_reg(0x0A, 0x0003);  /* Disable motor lock */

    delay_ms(5);

    if (!tmag5273_init()) {
        while (1);  /* Halt on sensor failure */
    }
    tmag5273_clear_por();

    /* Initialize position tracking with first reading */
    tmag_data_t sensor;
    tmag5273_read_all(&sensor);

    float initial_angle = angleLUT_get_angle(sensor.x_raw, sensor.y_raw);
    last_angle_crad = (int32_t)(initial_angle * 100.0f);
    absolute_position_crad = 0;  /* Start at zero */
    last_position_crad = 0;
    
    /* Initialize watchdog timer */
    last_valid_cmd_time = system_millis;
    
    /* Enable interrupts and start 10kHz timer */
    __asm__("cpsie i");
    tim21_setup();

    uint32_t last_time_us = 0;

    while (1) {
        /* Handle UART commands */
        uart_poll();
        
        /* Check communication timeout */
        check_timeout();
        
        /* Measure main loop time */
        uint32_t now_us = get_time_us();
        if (last_time_us) {
            looptime_us = now_us - last_time_us;
        }
        last_time_us = now_us;
    }
}
