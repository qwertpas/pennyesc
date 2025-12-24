#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "mct8316z.h"
#include "tmag5273.h"
#include "angleLUT.h"

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
#define RESPONSE_LEN        11  /* START + STATUS + int32 + int32 + CRC */

/* Status flags */
#define STATUS_POSITION_REACHED  0x01
#define STATUS_ERROR            0x02

/* Position control */
#define PI_CRAD             314     /* π in centiradians */
#define TWO_PI_CRAD         628     /* 2π in centiradians */
#define POSITION_DEADBAND_CRAD 5    /* ~3 degrees deadband */

/* Timeout */
#define COMM_TIMEOUT_MS     2000

/* Motor parameters */
#define POLE_PAIRS  6

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

/* Sensor data (shared with ISR) */
static volatile int16_t sensor_x = 0;
static volatile int16_t sensor_y = 0;
static volatile float current_angle_rad = 0;

/* Commutation */
static volatile int advance_deg = 130;

/* Debug/benchmark variables (visible in MCUViewer) */
volatile uint32_t isr_duration_us = 0;
volatile uint32_t isr_max_us = 0;
volatile uint32_t isr_count = 0;
volatile uint32_t isr_overrun_count = 0;
volatile uint32_t uart_rx_count = 0;
volatile uint32_t uart_crc_errors = 0;
volatile uint32_t looptime_us = 0;
volatile int32_t debug_position = 0;
volatile int32_t debug_velocity = 0;
volatile int debug_comm_step = 0;

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
    usart_set_baudrate(USART2, 921600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
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
    timer_set_period(TIM2, 799);  /* 40kHz PWM */
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_value(TIM2, TIM_OC4, 0);
    timer_enable_oc_output(TIM2, TIM_OC4);
    timer_enable_oc_preload(TIM2, TIM_OC4);
    timer_enable_counter(TIM2);
}

/* ============================================================================
 * 10kHz Timer Setup (TIM21 for encoder sampling)
 * ============================================================================ */

static void tim21_setup(void)
{
    /* TIM21 is a basic timer on STM32L0 */
    /* 32MHz / 32 = 1MHz timer clock, period 100 = 10kHz */
    timer_set_prescaler(TIM21, 31);  /* 32MHz / 32 = 1MHz */
    timer_set_period(TIM21, 99);     /* 1MHz / 100 = 10kHz */
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
    uint32_t start = get_time_us();
    timer_clear_flag(TIM21, TIM_SR_UIF);
    
    /* Read TMAG5273 sensor */
    tmag_data_t sensor;
    tmag5273_read_xyt(&sensor);
    sensor_x = sensor.x_raw;
    sensor_y = sensor.y_raw;
    
    /* Get angle from calibration LUT (returns radians) */
    float angle_rad = angleLUT_get_angle(sensor.x_raw, sensor.y_raw);
    current_angle_rad = angle_rad;
    
    /* Update multi-turn position */
    update_absolute_position(angle_rad);
    
    /* Velocity estimation (centiradians per second) */
    /* At 10kHz, delta_t = 0.0001s, so velocity = delta_pos * 10000 */
    static uint8_t vel_counter = 0;
    vel_counter++;
    if (vel_counter >= 10) {  /* Update velocity every 1ms (10 samples) */
        velocity_crads = (absolute_position_crad - last_position_crad) * 1000;
        last_position_crad = absolute_position_crad;
        vel_counter = 0;
    }
    
    /* Position control logic */
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
    
    /* Apply PWM duty */
    int16_t abs_duty = duty_to_apply < 0 ? -duty_to_apply : duty_to_apply;
    if (abs_duty > 799) abs_duty = 799;
    timer_set_oc_value(TIM2, TIM_OC4, abs_duty);
    
    /* Commutation - use predicted position for better tracking */
    int32_t pred_crad = absolute_position_crad;
    if (direction != 0) {
        /* Add velocity prediction for next step */
        pred_crad += (velocity_crads / 10000);  /* 0.1ms prediction */
    }
    
    int step = angle_to_step(pred_crad, direction > 0 ? advance_deg : -advance_deg);
    set_hall_outputs(step);
    debug_comm_step = step;
    
    /* Benchmark instrumentation */
    uint32_t elapsed = get_time_us() - start;
    isr_duration_us = elapsed;
    if (elapsed > isr_max_us) isr_max_us = elapsed;
    if (elapsed > 100) isr_overrun_count++;
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
    tx_buf[1] = status;
    
    /* Position (int32, little-endian) */
    int32_t pos = absolute_position_crad;
    tx_buf[2] = pos & 0xFF;
    tx_buf[3] = (pos >> 8) & 0xFF;
    tx_buf[4] = (pos >> 16) & 0xFF;
    tx_buf[5] = (pos >> 24) & 0xFF;
    
    /* Velocity (int32, little-endian) */
    int32_t vel = velocity_crads;
    tx_buf[6] = vel & 0xFF;
    tx_buf[7] = (vel >> 8) & 0xFF;
    tx_buf[8] = (vel >> 16) & 0xFF;
    tx_buf[9] = (vel >> 24) & 0xFF;
    
    /* CRC */
    tx_buf[10] = crc8_calculate(tx_buf, 10);
    
    /* Send */
    for (int i = 0; i < RESPONSE_LEN; i++) {
        usart_send_blocking(USART2, tx_buf[i]);
    }
}

static void process_command(void)
{
    uint8_t cmd = rx_buf[1];
    
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
                commanded_duty = 200;  /* Default duty if none set */
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
    
    while (usart_get_flag(USART2, USART_ISR_RXNE)) {
        uint8_t b = usart_recv(USART2);
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
        if (rx_idx >= 2) {
            switch (rx_buf[1]) {
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
    if ((system_millis - last_valid_cmd_time) > COMM_TIMEOUT_MS) {
        /* Safety: stop motor if no commands received */
        commanded_duty = 0;
        target_position_set = false;
        timer_set_oc_value(TIM2, TIM_OC4, 0);
    }
}

/* ============================================================================
 * Main
 * ============================================================================ */

int main(void)
{
    /* Startup delay for flash recovery */
    for (volatile int i = 0; i < 500000; i++);

    clock_setup();
    systick_setup();
    crc8_init();
    usart2_setup();
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
    tmag5273_read_xyt(&sensor);
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
