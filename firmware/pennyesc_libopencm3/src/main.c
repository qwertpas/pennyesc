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
#include "mct8316z.h"
#include "tmag5273.h"
#include "angleLUT.h"

/* Radians to degrees conversion factor */
#define RAD_TO_DEG  (180.0f / 3.14159265358979323846f)

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
}

static void systick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(31999);  /* 1ms at 32MHz */
    systick_interrupt_enable();
    systick_counter_enable();
    __asm__("cpsie i");
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

static void i2c1_setup(void)
{
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, GPIO6 | GPIO7);
    i2c_peripheral_disable(I2C1);
    I2C_TIMINGR(I2C1) = 0x00100413;  /* 400kHz Fast Mode */
    i2c_peripheral_enable(I2C1);
}

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
    /* Ensure step is 0-5 (handle negative values from modulo) */
    step = ((step % 6) + 6) % 6;
    const uint8_t *p = hall_pattern[step];
    if (p[0]) gpio_set(HALLA_PORT, HALLA_PIN); else gpio_clear(HALLA_PORT, HALLA_PIN);
    if (p[1]) gpio_set(HALLB_PORT, HALLB_PIN); else gpio_clear(HALLB_PORT, HALLB_PIN);
    if (p[2]) gpio_set(HALLC_PORT, HALLC_PIN); else gpio_clear(HALLC_PORT, HALLC_PIN);
}

/**
 * Calculate commutation step from mechanical angle (integer math)
 * @param mech_angle_deg  Mechanical angle in degrees (0-360)
 * @param advance_deg     Electrical advance angle for torque (positive = forward)
 * @return Commutation step 0-5
 */
static int angle_to_step(float mech_angle_deg, int advance_deg)
{
    /* Convert to integer tenths of degrees for precision without float math */
    int angle_x10 = (int)(mech_angle_deg * 10.0f);
    
    /* Electrical angle = mechanical × pole_pairs + advance (in tenths) */
    int elec_x10 = angle_x10 * POLE_PAIRS + advance_deg * 10;
    
    /* Normalize to 0-3600 range (0-360° in tenths) */
    elec_x10 = elec_x10 % 3600;
    if (elec_x10 < 0) elec_x10 += 3600;
    
    /* Each step covers 60° = 600 tenths */
    return elec_x10 / 600;
}

/* ============================================================================
 * Position/Velocity Estimator (handles angle wraparound)
 * Uses predictor-corrector with tunable gains
 * ============================================================================ */

/* Estimator state (in degrees, scaled x1000 for precision) */
static int32_t est_pos_x1000 = 0;      /* Estimated position × 1000 */
static int32_t est_vel_x1000 = 0;      /* Estimated velocity × 1000 (deg/loop) */

/* Estimator gains (0-1000 range, where 1000 = 1.0) */
volatile int est_pos_gain = 200;   /* Position correction gain (higher = trust sensor more) */
volatile int est_vel_gain = 50;    /* Velocity correction gain */

/**
 * Compute signed angular difference handling wraparound
 * Returns value in range [-180, 180) degrees × 1000
 */
static int32_t angle_diff_x1000(int32_t a, int32_t b)
{
    int32_t diff = a - b;
    while (diff > 180000) diff -= 360000;
    while (diff < -180000) diff += 360000;
    return diff;
}

/**
 * Update position/velocity estimate with new measurement
 * @param meas_deg  Measured angle in degrees (0-360)
 */
static void estimator_update(float meas_deg)
{
    int32_t meas_x1000 = (int32_t)(meas_deg * 1000.0f);
    
    /* Predict: propagate position using velocity */
    int32_t pred_pos = est_pos_x1000 + est_vel_x1000;
    
    /* Normalize predicted position to [0, 360000) */
    while (pred_pos >= 360000) pred_pos -= 360000;
    while (pred_pos < 0) pred_pos += 360000;
    
    /* Compute innovation (measurement - prediction), handling wraparound */
    int32_t innov = angle_diff_x1000(meas_x1000, pred_pos);
    
    /* Correct position: blend prediction with measurement */
    est_pos_x1000 = pred_pos + (innov * est_pos_gain) / 1000;
    
    /* Normalize */
    while (est_pos_x1000 >= 360000) est_pos_x1000 -= 360000;
    while (est_pos_x1000 < 0) est_pos_x1000 += 360000;
    
    /* Correct velocity: innovation indicates velocity error */
    est_vel_x1000 += (innov * est_vel_gain) / 1000;
    
    /* Clamp velocity to reasonable range (±36000 deg/loop = ±100 rev/loop max) */
    if (est_vel_x1000 > 36000) est_vel_x1000 = 36000;
    if (est_vel_x1000 < -36000) est_vel_x1000 = -36000;
}

/**
 * Get estimated position in degrees
 */
static float estimator_get_position(void)
{
    return est_pos_x1000 / 1000.0f;
}

/* Debug/tuning variables (accessible via debugger/MCUViewer) */
volatile int magx = 0;
volatile int magy = 0;
volatile float magangle = 0;      /* Raw sensor angle */
volatile float est_angle = 0;     /* Filtered/estimated angle */
volatile float est_velocity = 0;  /* Estimated velocity (deg/loop) */
volatile int magtemp = 0;
volatile int comm_step = 0;       /* Current commutation step (0-5) */
volatile int advance_deg = 130;    /* Electrical advance angle in degrees */
volatile uint32_t looptime_us = 0;

int main(void)
{
    /* Startup delay for flash recovery */
    for (volatile int i = 0; i < 500000; i++);

    clock_setup();
    systick_setup();
    usart2_setup();
    i2c1_setup();
    tim2_pwm_setup();
    motor_gpio_setup();
    mct8316z_init();

    delay_ms(5);

    mct8316z_set_pwm_mode_async_dig();

    // Set MTR_LOCK_MODE (Bits 1-0) to 11b (Disabled)
    // Address: 0x0A
    // Value: 0x0003 (This sets MODE to Disabled, keeping other defaults 0)
    mct8316z_write_reg(0x0A, 0x0003);

    delay_ms(5);


    timer_set_oc_value(TIM2, TIM_OC4, 100);  //out of 799

    if (!tmag5273_init()) {
        while (1);  /* Halt on sensor failure */
    }
    tmag5273_clear_por();

    tmag_data_t sensor;
    uint32_t last_time_us = 0;

    /* Initialize estimator with first reading */
    tmag5273_read_xyt(&sensor);
    magangle = angleLUT_get_angle(sensor.x_raw, sensor.y_raw) * RAD_TO_DEG;
    est_pos_x1000 = (int32_t)(magangle * 1000.0f);

    while (1) {
        /* Read magnetic sensor */
        tmag5273_read_xyt(&sensor);

        magx = sensor.x_raw;
        magy = sensor.y_raw;
        magtemp = (int)sensor.temp_degc;
        
        /* Get raw calibrated mechanical angle (0-360°) */
        magangle = angleLUT_get_angle(sensor.x_raw, sensor.y_raw) * RAD_TO_DEG;

        /* Update estimator with measurement */
        estimator_update(magangle);
        est_angle = estimator_get_position();
        est_velocity = est_vel_x1000 / 1000.0f;

        /* Predict position 1 loop ahead to compensate for sensor-to-output delay */
        /* predicted_pos = current_pos + velocity * 1_loop (velocity is in deg/loop) */
        int32_t pred_pos_x1000 = est_pos_x1000 + est_vel_x1000;
        while (pred_pos_x1000 >= 360000) pred_pos_x1000 -= 360000;
        while (pred_pos_x1000 < 0) pred_pos_x1000 += 360000;

        /* Six-step commutation based on PREDICTED rotor position */
        comm_step = angle_to_step(pred_pos_x1000 / 1000.0f, advance_deg);
        set_hall_outputs(comm_step);

        /* Measure loop time */
        uint32_t now_us = get_time_us();
        if (last_time_us) {
            looptime_us = now_us - last_time_us;
        }
        last_time_us = now_us;
    }
}
