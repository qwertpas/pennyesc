#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/syscfg.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "mct8316z.h"
#include "pennyesc_boot.h"
#include "pennyesc_calibration.h"
#include "pennyesc_frame.h"
#include "pennyesc_protocol.h"
#include "pennyesc_uart_update.h"
#include "tmag5273.h"

#ifndef PNY_ESC_ADDRESS
#define PNY_ESC_ADDRESS 1
#endif

#define ESC_ADDRESS PNY_ESC_ADDRESS

#if ESC_ADDRESS > 0xF
#error "ESC_ADDRESS must fit in 4 bits"
#endif

#define POLE_PAIRS 6

#define PWM_PERIOD 799
#define DUTY_LIMIT 799
#define DEFAULT_CONTROL_CLIP 150
#define POSITION_DEADBAND_TURN32 626
#define VELOCITY_DEADBAND_TURN32_PER_S 1043
#define CONTROL_GAIN_Q 8
#define CONTROL_GAIN_SCALE (1 << CONTROL_GAIN_Q)
#define CONTROL_TURN32_NUM 25
#define CONTROL_TURN32_SHIFT 26
#define ADVANCE_DEG 90
#define LEAD_DEFAULT_TURN16 ((int32_t)(((int32_t)ADVANCE_DEG * 65536 + 180) / 360))
#define REVERSE_HALL_PHASE_TURN16 32768
#define ADVANCE_MIN_DEG -180
#define ADVANCE_MAX_DEG 180
#define OBSERVER_LEAD_US 180
#define OBSERVER_LEAD_MIN_US -1000
#define OBSERVER_LEAD_MAX_US 1000
#define ISR_FREQ_HZ 10000
#define SENSOR_TICK_US (1000000u / ISR_FREQ_HZ)
#define SCHED_TIMER_PERIOD 0xffffu
#define SENSOR_STALE_US 10000u
#define SENSOR_START_RETRIES 4u
#define COMM_MIN_EVENT_US 2u
#define COMM_MAX_EVENT_US 60000u
#define VEL_UPDATE_SAMPLES (ISR_FREQ_HZ / 1000u)
#define COMM_VELOCITY_SAMPLES 5u

#define CAL_DUTY 100
#define CAL_SETTLE_MS 180u
#define CAL_SAMPLE_INTERVAL_MS 2u
#define CAL_SAMPLE_COUNT 16u
#define UART_FRAME_TIMEOUT_MS 10u
#define UART_APP_BAUD 2000000u
#define UART_DMA_RX_BUF_SIZE 64u
#define UART_DMA_RX_BUF_MASK (UART_DMA_RX_BUF_SIZE - 1u)
#define MCT_IDLE_CHECK_MS 500u
#define MCT_UART_IDLE_MS 10u

#define CAPTURE_MAX_DURATION_MS 2000u
#define MCT_IC_STATUS_NPOR (1u << 3)
#define MCT_EXPECTED_CONTROL2A (MCT8316Z_CONTROL2A_RESERVED | MCT8316Z_SDO_MODE_PUSH_PULL | MCT8316Z_PWM_MODE_SYNC_DIG)
#define MCT_EXPECTED_CONTROL3 MCT8316Z_CONTROL3_NO_REPORTS
#define MCT_EXPECTED_CONTROL4 (0x10u | MCT8316Z_OCP_MODE_DISABLED)
#define MCT_EXPECTED_CONTROL5 0x00u
#define MCT_EXPECTED_CONTROL6 0x00u
#define MCT_EXPECTED_CONTROL8 (MCT8316Z_MTR_LOCK_RETRY_5000MS | MCT8316Z_MTR_LOCK_TDET_5000MS | MCT8316Z_MTR_LOCK_DISABLED)
#define MCT_EXPECTED_CONTROL9 0x00u
#define MCT_EXPECTED_CONTROL10 0x00u

typedef struct {
    uint8_t reg;
    uint8_t value;
} mct_reg_value_t;

typedef struct {
    uint8_t reg;
    uint8_t value;
    uint8_t bad_mask;
} mct_check_t;

#define HALLA_PORT GPIOC
#define HALLA_PIN GPIO14
#define HALLB_PORT GPIOC
#define HALLB_PIN GPIO15
#define HALLC_PORT GPIOA
#define HALLC_PIN GPIO0
#define BRAKE_PORT GPIOA
#define BRAKE_PIN GPIO1

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

typedef struct {
    bool active;
    uint8_t index;
    uint8_t total_points;
    uint8_t sweep_dir;
    uint8_t sample_count;
    uint32_t next_ms;
    int32_t sum_x;
    int32_t sum_y;
    int32_t sum_z;
    int16_t min_x;
    int16_t min_y;
    int16_t max_x;
    int16_t max_y;
    pny_cal_point_payload_t points[PNY_CAL_POINTS_PER_SWEEP];
} calibration_state_t;

typedef struct {
    volatile bool active;
    volatile bool done;
    volatile uint16_t duration_ms;
    volatile uint16_t elapsed_ms;
    volatile uint16_t sample_hz;
    volatile uint16_t sample_period_ms;
    volatile uint16_t sample_ticks;
    volatile uint16_t sample_count;
    volatile uint16_t missed_count;
    volatile pny_capture_sample_t samples[PNY_CAPTURE_MAX_SAMPLES];
} capture_state_t;

typedef struct {
    volatile int32_t position_turn32;
} observer_state_t;

typedef struct {
    volatile uint8_t sector;
    volatile uint16_t next_sensor_tick;
    volatile uint16_t next_comm_tick;
    volatile uint16_t position_tick;
    int16_t last_duty;
} comm_scheduler_t;

typedef union {
    calibration_state_t cal;
    capture_state_t capture;
} work_state_t;

static volatile uint32_t system_millis;

static volatile int16_t sensor_x;
static volatile int16_t sensor_y;
static volatile int16_t sensor_z;
static volatile uint16_t current_angle_turn16;
static volatile int32_t absolute_position_turn32;
static volatile int32_t velocity_turn32_per_s;
static volatile int32_t comm_velocity_turn32_per_s;
static volatile int32_t position_zero_turn32;

static volatile bool position_initialized;
static volatile uint16_t last_angle_turn16;
static volatile bool sensor_ready;
static bool flash_fault;

static volatile uint8_t current_mode = PNY_MODE_IDLE;
static volatile bool target_position_set;
static volatile int32_t target_position_turn32;
static volatile int32_t target_velocity_turn32_per_s;
static volatile int16_t current_duty;
static volatile int8_t current_direction;
static volatile int16_t control_kp_q8;
static volatile int16_t control_kd_q8;
static volatile int16_t control_kv_q8;
static volatile int16_t control_kf;
static volatile int16_t control_clip = DEFAULT_CONTROL_CLIP;
static volatile bool report_speed;
static volatile int32_t current_lead_turn16 = LEAD_DEFAULT_TURN16;
static volatile int16_t observer_lead_us = OBSERVER_LEAD_US;
static volatile uint8_t observer_mode = PNY_OBSERVER_AB_FAST;
static uint32_t pending_reset_ms;

static volatile uint32_t isr_duration_us;
static volatile uint32_t isr_max_us;
static volatile uint32_t isr_overrun_count;
static volatile uint32_t sensor_i2c_us;
static volatile uint16_t sensor_i2c_start_us;
static volatile uint16_t sensor_i2c_end_us;
static uint16_t mct_fault_count;
static uint32_t next_mct_check_ms;
static uint32_t uart_overrun_errors;

static work_state_t work_state;
static observer_state_t observer_state;
static comm_scheduler_t comm_scheduler;
static int32_t comm_velocity_positions[COMM_VELOCITY_SAMPLES];
static uint16_t comm_velocity_ticks[COMM_VELOCITY_SAMPLES];
static uint8_t comm_velocity_index;
static uint8_t comm_velocity_count;

static const mct_reg_value_t mct_run_config[] = {
    {MCT8316Z_REG_CONTROL1, 0x03u},
    {MCT8316Z_REG_CONTROL2A, MCT_EXPECTED_CONTROL2A | MCT8316Z_CLR_FLT},
    {MCT8316Z_REG_CONTROL3, MCT_EXPECTED_CONTROL3},
    {MCT8316Z_REG_CONTROL4, MCT_EXPECTED_CONTROL4},
    {MCT8316Z_REG_CONTROL5, MCT_EXPECTED_CONTROL5},
    {MCT8316Z_REG_CONTROL6, MCT_EXPECTED_CONTROL6},
    {MCT8316Z_REG_CONTROL7, MCT8316Z_HALL_HYS_HIGH},
    {MCT8316Z_REG_CONTROL8, MCT_EXPECTED_CONTROL8},
    {MCT8316Z_REG_CONTROL9, MCT_EXPECTED_CONTROL9},
    {MCT8316Z_REG_CONTROL10, MCT_EXPECTED_CONTROL10},
    {MCT8316Z_REG_CONTROL2A, MCT_EXPECTED_CONTROL2A | MCT8316Z_CLR_FLT},
};

static const mct_check_t mct_expected_config[] = {
    {MCT8316Z_REG_CONTROL2A, MCT_EXPECTED_CONTROL2A, 1u << 1},
    {MCT8316Z_REG_CONTROL3, MCT_EXPECTED_CONTROL3, 1u << 2},
    {MCT8316Z_REG_CONTROL4, MCT_EXPECTED_CONTROL4, 1u << 3},
    {MCT8316Z_REG_CONTROL5, MCT_EXPECTED_CONTROL5, 1u << 7},
    {MCT8316Z_REG_CONTROL6, MCT_EXPECTED_CONTROL6, 1u << 7},
    {MCT8316Z_REG_CONTROL8, MCT_EXPECTED_CONTROL8, 1u << 6},
    {MCT8316Z_REG_CONTROL9, MCT_EXPECTED_CONTROL9, 1u << 7},
    {MCT8316Z_REG_CONTROL10, MCT_EXPECTED_CONTROL10, 1u << 7},
};

static const uint16_t comm_sector_start_turn16[] = {
    0x0000u,
    0x2aabu,
    0x5556u,
    0x8000u,
    0xaaabu,
    0xd556u,
    0x0000u,
};

#define cal_state work_state.cal
#define capture_state work_state.capture

static pny_frame_parser_t frame_parser;
static uint32_t uart_quiet_until_ms;
static uint32_t uart_last_frame_ms;
static volatile uint8_t uart_dma_rx[UART_DMA_RX_BUF_SIZE];
static uint16_t uart_dma_tail;
static bool sensor_run_mode;

void sys_tick_handler(void)
{
    system_millis++;
}

/* Hardware setup */
static void clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hsi16_32mhz);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_SYSCFG);
    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_TIM21);
    rcc_periph_clock_enable(RCC_DMA);
}

static void systick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(31999);
    systick_interrupt_enable();
    systick_counter_enable();
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = system_millis;
    while ((system_millis - start) < ms) {
    }
}

/* Small math helpers */
static int8_t position_target_direction(int32_t target, int32_t current, int32_t deadband)
{
    if (target > current) {
        if (current > INT32_MAX - deadband) {
            return 0;
        }
        return (target > current + deadband) ? 1 : 0;
    }
    if (target < current) {
        if (current < INT32_MIN + deadband) {
            return 0;
        }
        return (target < current - deadband) ? -1 : 0;
    }
    return 0;
}

static int32_t position_add_turn16_delta(int32_t position, int16_t delta)
{
    if (delta > 0 && position > INT32_MAX - delta) {
        return INT32_MAX;
    }
    if (delta < 0 && position < INT32_MIN - delta) {
        return INT32_MIN;
    }
    return position + delta;
}

static int16_t turn32_per_s_to_rpm(int32_t turn32_per_s)
{
    int32_t rpm = (turn32_per_s * 15) / 16384;
    if (rpm > INT16_MAX) {
        return INT16_MAX;
    }
    if (rpm < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)rpm;
}

static uint16_t approx_hypot_u16(uint32_t x, uint32_t y)
{
    uint32_t maxv = (x > y) ? x : y;
    uint32_t minv = (x > y) ? y : x;
    uint32_t value = maxv + (minv >> 1);
    return (value > 0xFFFFu) ? 0xFFFFu : (uint16_t)value;
}

static uint32_t abs_u32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

static void send_frame(uint8_t cmd, const void *payload, uint8_t payload_len)
{
    pny_frame_send((uint8_t)((ESC_ADDRESS << 4) | (cmd & 0x0Fu)), payload, payload_len);
}

/* UART, I2C, timer, and motor outputs */
static void uart_dma_setup(void)
{
    dma_channel_reset(DMA1, DMA_CHANNEL5);
    dma_set_channel_request(DMA1, DMA_CHANNEL5, 4);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t)&USART_RDR(USART2));
    dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)uart_dma_rx);
    dma_set_number_of_data(DMA1, DMA_CHANNEL5, UART_DMA_RX_BUF_SIZE);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL5);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL5);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);
    dma_set_priority(DMA1, DMA_CHANNEL5, DMA_CCR_PL_HIGH);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL5);
    uart_dma_tail = 0;
    dma_enable_channel(DMA1, DMA_CHANNEL5);
    usart_enable_rx_dma(USART2);
}

static void usart2_setup(uint32_t baud)
{
    usart_disable(USART2);
    dma_disable_channel(DMA1, DMA_CHANNEL5);
    gpio_set_af(GPIOA, GPIO_AF4, GPIO9 | GPIO10);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO9);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO10);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, GPIO9);

    usart_set_baudrate(USART2, baud);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_enable(USART2);
    uart_dma_setup();
}

static void i2c1_setup(void)
{
    gpio_set_af(GPIOB, GPIO_AF1, GPIO6 | GPIO7);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO6 | GPIO7);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, GPIO6 | GPIO7);
    SYSCFG_CFGR2 |= SYSCFG_CFGR2_I2C_PB6_FMP | SYSCFG_CFGR2_I2C_PB7_FMP | SYSCFG_CFGR2_I2C1_FMP;
    i2c_peripheral_disable(I2C1);
    I2C_TIMINGR(I2C1) = 0x00100107;
    i2c_peripheral_enable(I2C1);
    nvic_enable_irq(NVIC_I2C1_IRQ);
    nvic_set_priority(NVIC_I2C1_IRQ, 0);
}

static void tim2_pwm_setup(void)
{
    gpio_set_af(GPIOB, GPIO_AF5, GPIO1);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, 0);
    timer_set_period(TIM2, PWM_PERIOD);
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_value(TIM2, TIM_OC4, 0);
    timer_enable_oc_output(TIM2, TIM_OC4);
    timer_enable_oc_preload(TIM2, TIM_OC4);
    timer_enable_counter(TIM2);
}

static void tim21_setup(void)
{
    timer_set_prescaler(TIM21, 31);
    timer_set_period(TIM21, SCHED_TIMER_PERIOD);
    TIM_DIER(TIM21) &= ~(TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE);
    nvic_enable_irq(NVIC_TIM21_IRQ);
    nvic_set_priority(NVIC_TIM21_IRQ, 1);
    timer_enable_counter(TIM21);
}

static uint16_t sched_now_us(void)
{
    return (uint16_t)timer_get_counter(TIM21);
}

static void motor_gpio_setup(void)
{
    gpio_mode_setup(HALLA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HALLA_PIN);
    gpio_set_output_options(HALLA_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, HALLA_PIN);
    gpio_clear(HALLA_PORT, HALLA_PIN);

    gpio_mode_setup(HALLB_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HALLB_PIN);
    gpio_set_output_options(HALLB_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, HALLB_PIN);
    gpio_clear(HALLB_PORT, HALLB_PIN);

    gpio_mode_setup(HALLC_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, HALLC_PIN);
    gpio_set_output_options(HALLC_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, HALLC_PIN);
    gpio_clear(HALLC_PORT, HALLC_PIN);

    gpio_mode_setup(BRAKE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BRAKE_PIN);
    gpio_set_output_options(BRAKE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, BRAKE_PIN);
    gpio_clear(BRAKE_PORT, BRAKE_PIN);
}

static void set_hall_outputs_raw(uint8_t step)
{
    uint16_t pc_set = 0u;
    uint16_t pa_set = 0u;

    switch (step) {
    case 0:
        pc_set = HALLA_PIN | HALLB_PIN;
        break;
    case 1:
        pc_set = HALLA_PIN;
        break;
    case 2:
        pc_set = HALLA_PIN;
        pa_set = HALLC_PIN;
        break;
    case 3:
        pa_set = HALLC_PIN;
        break;
    case 4:
        pc_set = HALLB_PIN;
        pa_set = HALLC_PIN;
        break;
    default:
        pc_set = HALLB_PIN;
        break;
    }

    gpio_clear(GPIOC, HALLA_PIN | HALLB_PIN);
    gpio_clear(GPIOA, HALLC_PIN);
    gpio_set(GPIOC, pc_set);
    gpio_set(GPIOA, pa_set);
}

static void apply_pwm_duty(int16_t duty)
{
    uint16_t pwm = (duty < 0) ? (uint16_t)(-duty) : (uint16_t)duty;
    if (pwm > DUTY_LIMIT) {
        pwm = DUTY_LIMIT;
    }
    pwm = (uint16_t)(((uint32_t)pwm * PWM_PERIOD + (DUTY_LIMIT / 2u)) / DUTY_LIMIT);
    timer_set_oc_value(TIM2, TIM_OC4, pwm);
}

static void clear_motor_outputs(void)
{
    apply_pwm_duty(0);
    gpio_clear(HALLA_PORT, HALLA_PIN);
    gpio_clear(HALLB_PORT, HALLB_PIN);
    gpio_clear(HALLC_PORT, HALLC_PIN);
    current_duty = 0;
    current_direction = 0;
}

static void stop_motor_outputs(void)
{
    clear_motor_outputs();
    velocity_turn32_per_s = 0;
    comm_velocity_turn32_per_s = 0;
}

static void velocity_reset(int32_t position)
{
    for (uint8_t i = 0; i < COMM_VELOCITY_SAMPLES; i++) {
        comm_velocity_positions[i] = position;
        comm_velocity_ticks[i] = sched_now_us();
    }
    comm_velocity_index = 0;
    comm_velocity_count = 0;
    velocity_turn32_per_s = 0;
    comm_velocity_turn32_per_s = 0;
}

/* Position and sensor sampling */
static int32_t comm_velocity_update(int32_t position, uint16_t tick)
{
    comm_velocity_positions[comm_velocity_index] = position;
    comm_velocity_ticks[comm_velocity_index] = tick;
    comm_velocity_index++;
    if (comm_velocity_index >= COMM_VELOCITY_SAMPLES) {
        comm_velocity_index = 0;
    }
    if (comm_velocity_count < COMM_VELOCITY_SAMPLES) {
        comm_velocity_count++;
        return comm_velocity_turn32_per_s;
    }

    uint8_t oldest = comm_velocity_index;
    uint8_t i0 = oldest;
    uint8_t i1 = oldest + 1u;
    uint8_t i3 = oldest + 3u;
    uint8_t i4 = oldest + 4u;
    if (i1 >= COMM_VELOCITY_SAMPLES) {
        i1 -= COMM_VELOCITY_SAMPLES;
    }
    if (i3 >= COMM_VELOCITY_SAMPLES) {
        i3 -= COMM_VELOCITY_SAMPLES;
    }
    if (i4 >= COMM_VELOCITY_SAMPLES) {
        i4 -= COMM_VELOCITY_SAMPLES;
    }
    uint16_t dt_us = (uint16_t)(comm_velocity_ticks[i4] - comm_velocity_ticks[i0]);
    if (dt_us == 0u || dt_us >= SENSOR_STALE_US) {
        return comm_velocity_turn32_per_s;
    }

    int32_t sum = (2 * (comm_velocity_positions[i4] - comm_velocity_positions[i0])) +
                  (comm_velocity_positions[i3] - comm_velocity_positions[i1]);
    return ((sum * 2000) / (int32_t)(5u * dt_us)) * 1000;
}

static void update_position_from_angle(uint16_t angle_turn16)
{
    if (!position_initialized) {
        last_angle_turn16 = angle_turn16;
        absolute_position_turn32 = angle_turn16;
        velocity_reset(absolute_position_turn32);
        position_initialized = true;
        return;
    }

    int16_t delta = (int16_t)(angle_turn16 - last_angle_turn16);
    absolute_position_turn32 = position_add_turn16_delta(absolute_position_turn32, delta);
    last_angle_turn16 = angle_turn16;
}

static int32_t position_from_zero(int32_t position)
{
    int64_t delta = (int64_t)position - position_zero_turn32;
    if (delta > INT32_MAX) {
        return INT32_MAX;
    }
    if (delta < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)delta;
}

static bool position_target_reached(void)
{
    return target_position_set &&
           position_target_direction(
               target_position_turn32,
               position_from_zero(absolute_position_turn32),
               POSITION_DEADBAND_TURN32) == 0;
}

static bool sensor_set_full_mode(void)
{
    if (!sensor_run_mode) {
        return true;
    }
    if (!tmag5273_set_mode(TMAG5273_MODE_FULL_XYZ)) {
        return false;
    }
    sensor_run_mode = false;
    return true;
}

static bool sensor_init_full_mode(void)
{
    for (uint8_t i = 0; i < SENSOR_START_RETRIES; i++) {
        if (tmag5273_init()) {
            tmag5273_clear_por();
            sensor_ready = true;
            sensor_run_mode = false;
            return true;
        }
        delay_ms(1);
    }
    sensor_ready = false;
    sensor_run_mode = false;
    return false;
}

static bool sensor_set_run_mode(void)
{
    if (sensor_run_mode) {
        return true;
    }
    if (!tmag5273_set_mode(TMAG5273_MODE_FAST_XY)) {
        return false;
    }
    sensor_run_mode = true;
    return true;
}

static void refresh_sensor_xyz(bool update_position)
{
    int16_t x;
    int16_t y;
    int16_t z;

    if (!tmag5273_read_xyz_fast(&x, &y, &z)) {
        sensor_ready = false;
        current_angle_turn16 = 0;
        return;
    }
    sensor_ready = true;
    sensor_x = x;
    sensor_y = y;
    sensor_z = z;

    if (pennyesc_calibration_valid()) {
        uint16_t angle = pennyesc_calibration_angle_turn16(x, y);
        current_angle_turn16 = angle;
        if (update_position) {
            update_position_from_angle(angle);
        }
    } else {
        current_angle_turn16 = 0;
    }
}

static bool refresh_sensor_xy(bool update_position)
{
    int16_t x;
    int16_t y;

    if (!tmag5273_read_xy_fast(&x, &y)) {
        sensor_ready = false;
        current_angle_turn16 = 0;
        return false;
    }
    sensor_ready = true;
    sensor_x = x;
    sensor_y = y;
    sensor_z = 0;

    uint16_t angle = pennyesc_calibration_angle_turn16(x, y);
    current_angle_turn16 = angle;
    if (update_position) {
        update_position_from_angle(angle);
    }
    return true;
}

static void apply_sensor_xy_sample(const tmag5273_xy_sample_t *sample, bool update_position)
{
    sensor_ready = true;
    sensor_x = sample->x;
    sensor_y = sample->y;
    sensor_z = 0;
    sensor_i2c_start_us = sample->start_phase_us;
    sensor_i2c_end_us = sample->end_phase_us;
    sensor_i2c_us = (uint16_t)(sample->end_phase_us - sample->start_phase_us);
    uint16_t angle = pennyesc_calibration_angle_turn16(sample->x, sample->y);
    current_angle_turn16 = angle;
    if (update_position) {
        update_position_from_angle(angle);
    }
}

static int32_t lead_deg_to_turn16(int16_t lead_deg)
{
    int32_t scaled = (int32_t)lead_deg * 65536;
    if (scaled >= 0) {
        return (scaled + 180) / 360;
    }
    return -((-scaled + 180) / 360);
}

/* Commutation scheduler */
static int32_t observer_position_at_tick(uint16_t tick)
{
    int32_t dt_us = (int16_t)(tick - comm_scheduler.position_tick);
    dt_us += observer_lead_us;
    return observer_state.position_turn32 +
           (comm_velocity_turn32_per_s / 1000) * dt_us / 1000;
}

static int32_t clamp_position_error(int32_t error)
{
    if (error > 32768) {
        return 32768;
    }
    if (error < -32768) {
        return -32768;
    }
    return error;
}

static void observer_ab_update(int32_t measured_position, uint16_t sample_tick, uint8_t alpha_div, uint16_t beta_div)
{
    uint16_t dt_us = (uint16_t)(sample_tick - comm_scheduler.position_tick);
    if (dt_us == 0u || dt_us >= SENSOR_STALE_US) {
        observer_state.position_turn32 = measured_position;
        return;
    }

    int32_t predicted = observer_state.position_turn32 +
                        (comm_velocity_turn32_per_s / 1000) * (int32_t)dt_us / 1000;
    int32_t error = clamp_position_error(measured_position - predicted);
    observer_state.position_turn32 = predicted + (error / alpha_div);
    int32_t correction = ((error * 1000) / (int32_t)dt_us) * 1000;
    comm_velocity_turn32_per_s += correction / beta_div;
}

static int32_t commutation_phase_at_tick(uint16_t tick, int direction)
{
    int32_t rotor_electrical = observer_position_at_tick(tick) * POLE_PAIRS;
    int32_t phase = rotor_electrical +
                    pennyesc_calibration_commutation_alignment_turn16() +
                    current_lead_turn16;
    if (direction < 0) {
        phase -= REVERSE_HALL_PHASE_TURN16;
    }
    return phase;
}

static uint8_t comm_sector_at_tick(uint16_t tick, int direction)
{
    uint16_t phase = (uint16_t)commutation_phase_at_tick(tick, direction);
    return (uint8_t)(((uint32_t)phase * 6u) >> 16);
}

static uint16_t comm_sector_start_phase(uint8_t sector)
{
    return comm_sector_start_turn16[sector];
}

static uint8_t comm_scheduler_schedule_sector_event(uint16_t now, int direction)
{
    int32_t electrical_velocity = comm_velocity_turn32_per_s * POLE_PAIRS;
    if (direction == 0 || electrical_velocity == 0) {
        TIM_DIER(TIM21) &= ~TIM_DIER_CC2IE;
        timer_clear_flag(TIM21, TIM_SR_CC2IF | TIM_SR_CC2OF);
        return 0xffu;
    }

    uint32_t speed = abs_u32(electrical_velocity);
    uint16_t phase = (uint16_t)commutation_phase_at_tick(now, direction);
    uint8_t sector = (uint8_t)(((uint32_t)phase * 6u) >> 16);
    uint16_t phase_delta;

    if (electrical_velocity > 0) {
        uint16_t boundary = comm_sector_start_phase((uint8_t)(sector + 1u));
        phase_delta = (uint16_t)(boundary - phase);
    } else {
        uint16_t boundary = (uint16_t)(comm_sector_start_phase(sector) - 1u);
        phase_delta = (uint16_t)(phase - boundary);
    }
    if (phase_delta == 0u) {
        phase_delta = 1u;
    }

    uint32_t speed_k = speed / 1000u;
    if (speed_k == 0u) {
        TIM_DIER(TIM21) &= ~TIM_DIER_CC2IE;
        timer_clear_flag(TIM21, TIM_SR_CC2IF | TIM_SR_CC2OF);
        return 0xffu;
    }
    uint32_t dt_us = (((uint32_t)phase_delta * 1000u) + speed_k - 1u) / speed_k;
    if (dt_us < COMM_MIN_EVENT_US) {
        dt_us = COMM_MIN_EVENT_US;
    }
    if (dt_us > COMM_MAX_EVENT_US) {
        TIM_DIER(TIM21) &= ~TIM_DIER_CC2IE;
        timer_clear_flag(TIM21, TIM_SR_CC2IF | TIM_SR_CC2OF);
        return 0xffu;
    }

    comm_scheduler.next_comm_tick = (uint16_t)(now + (uint16_t)dt_us);
    TIM_CCR2(TIM21) = comm_scheduler.next_comm_tick;
    timer_clear_flag(TIM21, TIM_SR_CC2IF | TIM_SR_CC2OF);
    TIM_DIER(TIM21) |= TIM_DIER_CC2IE;
    return sector;
}

static void comm_scheduler_stop(void)
{
    TIM_DIER(TIM21) &= ~(TIM_DIER_CC1IE | TIM_DIER_CC2IE);
    comm_scheduler.sector = 0xffu;
    comm_scheduler.last_duty = INT16_MIN;
    tmag5273_async_cancel();
}

static void comm_scheduler_start(void)
{
    uint16_t now = sched_now_us();
    comm_scheduler.sector = 0xffu;
    comm_scheduler.position_tick = now;
    comm_scheduler.last_duty = INT16_MIN;
    comm_scheduler.next_sensor_tick = (uint16_t)(now + SENSOR_TICK_US);
    comm_scheduler.next_comm_tick = now;
    tmag5273_async_start_xy(now);
    TIM_CCR1(TIM21) = comm_scheduler.next_sensor_tick;
    timer_clear_flag(TIM21, TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC1OF | TIM_SR_CC2OF | TIM_SR_UIF);
    TIM_DIER(TIM21) |= TIM_DIER_CC1IE;
}

static bool sensor_start_read(void)
{
    for (uint8_t i = 0; i < SENSOR_START_RETRIES; i++) {
        if (!sensor_ready && !sensor_init_full_mode()) {
            delay_ms(1);
            continue;
        }
        if (sensor_set_run_mode() && refresh_sensor_xy(false)) {
            return true;
        }
        sensor_ready = false;
        delay_ms(1);
    }
    return false;
}

/* Run readiness */
static void run_stop(void)
{
    capture_state.active = false;
    comm_scheduler_stop();
    if (current_mode == PNY_MODE_RUN) {
        current_mode = PNY_MODE_IDLE;
    }
    stop_motor_outputs();
    next_mct_check_ms = system_millis;
}

static uint8_t run_ready(void)
{
    if (!sensor_ready) {
        if (!sensor_start_read()) {
            return PNY_RESULT_BAD_STATE;
        }
    }
    if (!pennyesc_calibration_valid()) {
        return PNY_RESULT_NOT_CALIBRATED;
    }
    if (current_mode == PNY_MODE_CAL) {
        return PNY_RESULT_BUSY;
    }
    return PNY_RESULT_OK;
}

/* MCT8316Z recovery */
static void mct_apply_config(void)
{
    for (uint8_t i = 0; i < sizeof(mct_run_config) / sizeof(mct_run_config[0]); i++) {
        mct8316z_write_reg(mct_run_config[i].reg, mct_run_config[i].value);
    }
}

static uint8_t mct_config_bad_mask(void)
{
    uint8_t bad = 0u;

    uint8_t ic_status = (uint8_t)mct8316z_read_reg(MCT8316Z_REG_IC_STATUS);
    if ((ic_status & MCT_IC_STATUS_NPOR) == 0u) {
        bad |= 1u << 0;
    }

    for (uint8_t i = 0; i < sizeof(mct_expected_config) / sizeof(mct_expected_config[0]); i++) {
        uint8_t reg = mct_expected_config[i].reg;
        uint8_t value = (uint8_t)mct8316z_read_reg(reg);
        if (value != mct_expected_config[i].value) {
            bad |= mct_expected_config[i].bad_mask;
        }
    }

    uint8_t control7 = (uint8_t)mct8316z_read_reg(MCT8316Z_REG_CONTROL7);
    if ((control7 & MCT8316Z_HALL_HYS_HIGH) != MCT8316Z_HALL_HYS_HIGH) {
        bad |= 1u << 4;
    }
    if ((control7 & (MCT8316Z_HALL_HYS_HIGH | MCT8316Z_DIR_REVERSE)) != MCT8316Z_HALL_HYS_HIGH) {
        bad |= 1u << 5;
    }

    return bad;
}

static void mct_recover_if_needed(void)
{
    if (mct_config_bad_mask() == 0u) {
        return;
    }

    bool restart_timer = current_mode == PNY_MODE_RUN;
    if (restart_timer) {
        comm_scheduler_stop();
    }
    apply_pwm_duty(0);
    delay_ms(2);

    if (mct_config_bad_mask() == 0u) {
        if (restart_timer) {
            comm_scheduler_start();
        }
        return;
    }

    if (mct_fault_count != UINT16_MAX) {
        mct_fault_count++;
    }

    mct_apply_config();
    delay_ms(2);

    if (restart_timer) {
        comm_scheduler_start();
    }
}

static bool uart_rx_pending(void)
{
    uint16_t head = (uint16_t)((UART_DMA_RX_BUF_SIZE - dma_get_number_of_data(DMA1, DMA_CHANNEL5)) & UART_DMA_RX_BUF_MASK);
    return head != uart_dma_tail || ((USART_ISR(USART2) & USART_ISR_ORE) != 0u);
}

/* Control and run state */
static void control_disable(void)
{
    target_position_set = false;
    control_kp_q8 = 0;
    control_kd_q8 = 0;
    control_kv_q8 = 0;
    control_kf = 0;
    report_speed = false;
}

static int32_t control_term(int16_t gain_q8, int32_t error_turn32)
{
    return (int32_t)(((int64_t)gain_q8 * error_turn32 * CONTROL_TURN32_NUM) >> CONTROL_TURN32_SHIFT);
}

static int16_t clamp_duty_i32(int32_t duty)
{
    int16_t clip = control_clip;
    if (clip < 0) {
        clip = (int16_t)(-clip);
    }
    if (clip > DUTY_LIMIT) {
        clip = DUTY_LIMIT;
    }
    if (duty > clip) {
        return clip;
    }
    if (duty < -clip) {
        return (int16_t)(-clip);
    }
    return (int16_t)duty;
}

static int16_t control_duty(void)
{
    int32_t position_error = target_position_turn32 - position_from_zero(absolute_position_turn32);
    int32_t velocity_error = target_velocity_turn32_per_s - velocity_turn32_per_s;
    int32_t duty =
        control_term(control_kp_q8, position_error) +
        control_term(control_kd_q8, velocity_error) +
        control_term(control_kv_q8, target_velocity_turn32_per_s) +
        control_kf;
    return clamp_duty_i32(duty);
}

static void control_set_duty(int16_t duty, int16_t clip)
{
    target_position_set = false;
    control_kp_q8 = 0;
    control_kd_q8 = 0;
    control_kv_q8 = 0;
    control_kf = duty;
    control_clip = clip;
}

static void get_drive_command(int16_t *duty, int8_t *direction)
{
    *duty = control_duty();
    *direction = 0;

    if (*duty > 0) {
        *direction = 1;
    } else if (*duty < 0) {
        *direction = -1;
    }
}

static void run_begin(void)
{
    if (!sensor_start_read()) {
        sensor_ready = false;
        current_mode = PNY_MODE_IDLE;
        stop_motor_outputs();
        return;
    }
    update_position_from_angle(current_angle_turn16);
    velocity_reset(absolute_position_turn32);
    observer_state.position_turn32 = absolute_position_turn32;

    int16_t duty;
    int8_t direction;
    get_drive_command(&duty, &direction);

    if (duty == 0 || direction == 0) {
        clear_motor_outputs();
        current_mode = PNY_MODE_RUN;
        comm_scheduler_start();
        return;
    }

    mct_recover_if_needed();
    next_mct_check_ms = system_millis + MCT_IDLE_CHECK_MS;
    current_duty = duty;
    current_direction = direction;
    comm_scheduler.position_tick = sched_now_us();
    uint8_t sector = comm_sector_at_tick(comm_scheduler.position_tick, direction);
    set_hall_outputs_raw(sector);
    apply_pwm_duty(duty);
    current_mode = PNY_MODE_RUN;
    comm_scheduler_start();
    comm_scheduler.sector = sector;
    comm_scheduler_schedule_sector_event(sched_now_us(), direction);
}

static uint8_t control_apply(void)
{
    if (current_mode == PNY_MODE_CAL) {
        return PNY_RESULT_BUSY;
    }

    bool active = report_speed || control_kp_q8 != 0 || control_kd_q8 != 0 || control_kv_q8 != 0 || control_kf != 0;
    if (!active) {
        run_stop();
        current_duty = 0;
        current_direction = 0;
        return PNY_RESULT_OK;
    }

    if (current_mode != PNY_MODE_RUN) {
        uint8_t result = run_ready();
        if (result != PNY_RESULT_OK) {
            return result;
        }
        run_begin();
        if (current_mode != PNY_MODE_RUN) {
            return PNY_RESULT_BAD_STATE;
        }
    }
    return PNY_RESULT_OK;
}

static void control_restart_poll(void)
{
    bool active = report_speed || control_kp_q8 != 0 || control_kd_q8 != 0 || control_kv_q8 != 0 || control_kf != 0;
    if (!active || current_mode != PNY_MODE_IDLE) {
        return;
    }

    run_begin();
    if (current_mode != PNY_MODE_RUN) {
        stop_motor_outputs();
    }
}

static void mct_idle_check(void)
{
    uint32_t now = system_millis;

    if ((int32_t)(now - next_mct_check_ms) < 0) {
        return;
    }
    next_mct_check_ms = now + MCT_IDLE_CHECK_MS;
    if (current_mode != PNY_MODE_IDLE) {
        return;
    }
    if ((uint32_t)(now - uart_last_frame_ms) < MCT_UART_IDLE_MS || uart_rx_pending()) {
        next_mct_check_ms = now + MCT_UART_IDLE_MS;
        return;
    }

    mct_recover_if_needed();
}

/* Status and boot update */
static uint8_t current_faults(void)
{
    uint8_t faults = 0u;
    if (!sensor_ready) {
        faults |= PNY_FAULT_SENSOR;
    }
    if (!pennyesc_calibration_valid()) {
        faults |= PNY_FAULT_UNCALIBRATED;
    }
    if (flash_fault) {
        faults |= PNY_FAULT_FLASH;
    }
    return faults;
}

static uint8_t current_flags(uint8_t faults)
{
    uint8_t flags = 0u;
    if (pennyesc_calibration_valid()) {
        flags |= PNY_FLAG_CAL_VALID;
    }
    if (current_mode == PNY_MODE_CAL) {
        flags |= PNY_FLAG_BUSY;
    }
    if (sensor_ready) {
        flags |= PNY_FLAG_SENSOR_OK;
    }
    if (position_target_reached()) {
        flags |= PNY_FLAG_POSITION_REACHED;
    }
    if (faults != 0u) {
        flags |= PNY_FLAG_FAULT;
    }
    return flags;
}

static void fill_status_payload(pny_status_payload_t *payload, uint8_t result)
{
    tmag5273_stats_t tmag_stats;
    tmag5273_get_stats(&tmag_stats);

    payload->result = result;
    payload->mode = current_mode;
    payload->faults = current_faults();
    payload->flags = current_flags(payload->faults);
    payload->x = sensor_x;
    payload->y = sensor_y;
    payload->z = sensor_z;
    payload->angle_turn16 = current_angle_turn16;
    payload->position_turn32 = position_from_zero(absolute_position_turn32);
    payload->velocity_turn32_per_s = velocity_turn32_per_s;
    payload->duty = current_duty;
    payload->mct_fault_count = mct_fault_count;
    payload->isr_us = (uint16_t)isr_duration_us;
    payload->isr_max_us = (uint16_t)isr_max_us;
    payload->i2c_us = (uint16_t)sensor_i2c_us;
    payload->i2c_start_us = sensor_i2c_start_us;
    payload->i2c_end_us = sensor_i2c_end_us;
    payload->isr_overrun_count = isr_overrun_count;
    payload->i2c_timeout_count = tmag_stats.timeout_count;
    payload->i2c_nack_count = tmag_stats.nack_count;
    payload->i2c_recover_count = tmag_stats.recover_count;
    payload->uart_overrun_errors = uart_overrun_errors;
    payload->tmag_sample_count = tmag_stats.sample_count;
    payload->tmag_sample_dt_us = tmag_stats.sample_dt_us;
}

static void refresh_status_sensor(void)
{
    if (current_mode != PNY_MODE_IDLE) {
        return;
    }

    for (uint8_t i = 0; i < SENSOR_START_RETRIES; i++) {
        if ((!sensor_ready && !sensor_init_full_mode()) || !sensor_set_full_mode()) {
            sensor_ready = false;
            continue;
        }
        refresh_sensor_xyz(true);
        if (sensor_ready) {
            return;
        }
    }
}

static void fill_update_status(pny_status_payload_t *payload)
{
    refresh_status_sensor();
    fill_status_payload(payload, PNY_RESULT_OK);
}

static uint8_t prepare_update_boot(void)
{
    if (current_mode == PNY_MODE_CAL) {
        return PNY_RESULT_BUSY;
    }

    control_disable();
    run_stop();
    return PNY_RESULT_OK;
}

/* Calibration */
static uint8_t cal_point_step_index(uint8_t point_index, uint8_t sweep_dir)
{
    if (sweep_dir == 0u) {
        return point_index;
    }
    return (uint8_t)(PNY_CAL_POINTS_PER_SWEEP - 1u - point_index);
}

static void calibration_begin_sample_window(void)
{
    cal_state.sample_count = 0;
    cal_state.sum_x = 0;
    cal_state.sum_y = 0;
    cal_state.sum_z = 0;
    cal_state.min_x = 32767;
    cal_state.min_y = 32767;
    cal_state.max_x = -32768;
    cal_state.max_y = -32768;
}

static void calibration_arm_point(uint8_t point_index)
{
    uint8_t step = cal_point_step_index(point_index, cal_state.sweep_dir);
    mct_recover_if_needed();
    set_hall_outputs_raw((uint8_t)(step % 6u));
    apply_pwm_duty(CAL_DUTY);
    calibration_begin_sample_window();
    cal_state.next_ms = system_millis + CAL_SETTLE_MS;
}

static void calibration_finish_point(void)
{
    uint8_t point_index = cal_state.index;
    pny_cal_point_payload_t *point = &cal_state.points[point_index];
    int32_t avg_x = cal_state.sum_x / CAL_SAMPLE_COUNT;
    int32_t avg_y = cal_state.sum_y / CAL_SAMPLE_COUNT;
    int32_t avg_z = cal_state.sum_z / CAL_SAMPLE_COUNT;

    point->result = PNY_RESULT_OK;
    point->index = point_index;
    point->step_index = cal_point_step_index(point_index, cal_state.sweep_dir);
    point->sweep_dir = cal_state.sweep_dir;
    point->x = (int16_t)avg_x;
    point->y = (int16_t)avg_y;
    point->z = (int16_t)avg_z;
    point->xy_radius = approx_hypot_u16(abs_u32(avg_x), abs_u32(avg_y));
    point->sample_spread = approx_hypot_u16(
        (uint32_t)(cal_state.max_x - cal_state.min_x),
        (uint32_t)(cal_state.max_y - cal_state.min_y));
    point->duty = CAL_DUTY;

    cal_state.index++;
    if (cal_state.index >= cal_state.total_points) {
        cal_state.active = false;
        current_mode = PNY_MODE_IDLE;
        stop_motor_outputs();
        refresh_sensor_xyz(false);
        return;
    }

    calibration_arm_point(cal_state.index);
}

static uint8_t calibration_start(uint8_t sweep_dir)
{
    if (current_mode != PNY_MODE_IDLE) {
        return PNY_RESULT_BAD_STATE;
    }
    if (sweep_dir > 1u) {
        return PNY_RESULT_BAD_ARG;
    }

    memset(&cal_state, 0, sizeof(cal_state));
    control_disable();
    stop_motor_outputs();
    if (!sensor_init_full_mode()) {
        return PNY_RESULT_BAD_STATE;
    }
    mct_apply_config();
    delay_ms(2);
    current_mode = PNY_MODE_CAL;
    cal_state.active = true;
    cal_state.total_points = PNY_CAL_POINTS_PER_SWEEP;
    cal_state.sweep_dir = sweep_dir;
    calibration_arm_point(0);
    return PNY_RESULT_OK;
}

static void calibration_poll(void)
{
    if (!cal_state.active || current_mode != PNY_MODE_CAL) {
        return;
    }
    if ((int32_t)(system_millis - cal_state.next_ms) < 0) {
        return;
    }

    if (cal_state.sample_count == 0u) {
        cal_state.next_ms = system_millis;
    }

    int16_t x;
    int16_t y;
    int16_t z;
    if (!tmag5273_read_xyz_fast(&x, &y, &z)) {
        sensor_ready = false;
        cal_state.active = false;
        control_disable();
        current_mode = PNY_MODE_IDLE;
        stop_motor_outputs();
        return;
    }

    sensor_x = x;
    sensor_y = y;
    sensor_z = z;
    if (pennyesc_calibration_valid()) {
        current_angle_turn16 = pennyesc_calibration_angle_turn16(x, y);
    } else {
        current_angle_turn16 = 0;
    }

    cal_state.sum_x += x;
    cal_state.sum_y += y;
    cal_state.sum_z += z;
    if (x < cal_state.min_x) {
        cal_state.min_x = x;
    }
    if (x > cal_state.max_x) {
        cal_state.max_x = x;
    }
    if (y < cal_state.min_y) {
        cal_state.min_y = y;
    }
    if (y > cal_state.max_y) {
        cal_state.max_y = y;
    }

    cal_state.sample_count++;
    cal_state.next_ms = system_millis + CAL_SAMPLE_INTERVAL_MS;

    if (cal_state.sample_count >= CAL_SAMPLE_COUNT) {
        calibration_finish_point();
    }
}

/* Capture and commutation ISR */
static void capture_tick_1ms(void)
{
    if (!capture_state.active) {
        return;
    }

    int16_t rpm = turn32_per_s_to_rpm(velocity_turn32_per_s);
    capture_state.elapsed_ms++;

    capture_state.sample_ticks++;
    if (capture_state.sample_ticks >= capture_state.sample_period_ms) {
        capture_state.sample_ticks = 0;
        uint16_t index = capture_state.sample_count;
        if (index < PNY_CAPTURE_MAX_SAMPLES) {
            capture_state.samples[index].angle_turn16 = current_angle_turn16;
            capture_state.samples[index].rpm = rpm;
            capture_state.sample_count = (uint16_t)(index + 1u);
        } else if (capture_state.missed_count != UINT16_MAX) {
            capture_state.missed_count++;
        }
    }

    if (capture_state.elapsed_ms >= capture_state.duration_ms) {
        capture_state.active = false;
        capture_state.done = true;
        control_disable();
    }
}

static void comm_stop_from_isr(void)
{
    current_mode = PNY_MODE_IDLE;
    comm_scheduler_stop();
    stop_motor_outputs();
}

static void comm_control_tick(uint16_t now)
{
    int16_t duty;
    int8_t direction;
    get_drive_command(&duty, &direction);

    current_duty = duty;
    if (duty == 0 || direction == 0) {
        clear_motor_outputs();
        comm_scheduler.last_duty = INT16_MIN;
        comm_scheduler.sector = 0xffu;
        if (!capture_state.active && !report_speed &&
            control_kp_q8 == 0 && control_kd_q8 == 0 && control_kv_q8 == 0 && control_kf == 0) {
            current_mode = PNY_MODE_IDLE;
            comm_scheduler_stop();
        }
        return;
    }

    if (current_direction != 0 && direction != current_direction) {
        current_mode = PNY_MODE_IDLE;
        current_duty = 0;
        current_direction = 0;
        comm_scheduler_stop();
        stop_motor_outputs();
        return;
    }
    current_direction = direction;

    uint8_t sector = comm_sector_at_tick(now, direction);
    if (comm_scheduler.sector > 5u || sector != comm_scheduler.sector) {
        set_hall_outputs_raw(sector);
        comm_scheduler.sector = sector;
    }

    if (duty != comm_scheduler.last_duty) {
        apply_pwm_duty(duty);
        comm_scheduler.last_duty = duty;
    }
}

static void comm_sensor_tick(uint16_t scheduled_tick)
{
    static uint8_t vel_counter;
    uint16_t now = sched_now_us();
    uint16_t next = scheduled_tick;

    do {
        next = (uint16_t)(next + SENSOR_TICK_US);
    } while ((int16_t)(next - now) <= 0);
    comm_scheduler.next_sensor_tick = next;
    TIM_CCR1(TIM21) = next;

    int32_t last_position = absolute_position_turn32;
    uint16_t last_tick = comm_scheduler.position_tick;
    tmag5273_xy_sample_t sample;
    bool have_sample = tmag5273_async_take_xy(&sample);

    if (have_sample) {
        apply_sensor_xy_sample(&sample, true);
        uint16_t sample_tick = sensor_i2c_start_us;
        uint16_t sample_dt = (uint16_t)(sample_tick - last_tick);
        int32_t sample_velocity = comm_velocity_turn32_per_s;
        if (sample_dt > 0u && sample_dt < SENSOR_STALE_US) {
            sample_velocity = ((absolute_position_turn32 - last_position) * 1000) / (int32_t)sample_dt * 1000;
        }
        int32_t secant_velocity = comm_velocity_update(absolute_position_turn32, sample_tick);
        bool velocity_updated = false;
        switch (observer_mode) {
        case PNY_OBSERVER_AB_FAST:
            observer_ab_update(absolute_position_turn32, sample_tick, 8u, 128u);
            velocity_updated = true;
            break;
        case PNY_OBSERVER_AB_MID:
            observer_ab_update(absolute_position_turn32, sample_tick, 12u, 192u);
            velocity_updated = true;
            break;
        case PNY_OBSERVER_AB_SLOW:
            observer_ab_update(absolute_position_turn32, sample_tick, 16u, 256u);
            velocity_updated = true;
            break;
        default:
            observer_state.position_turn32 = absolute_position_turn32;
            break;
        }

        if (!velocity_updated) {
            if (observer_mode == PNY_OBSERVER_SECANT && comm_velocity_count >= COMM_VELOCITY_SAMPLES) {
                comm_velocity_turn32_per_s = secant_velocity;
            } else if (observer_mode == PNY_OBSERVER_SECANT_LP2 && comm_velocity_count >= COMM_VELOCITY_SAMPLES) {
                comm_velocity_turn32_per_s += (secant_velocity - comm_velocity_turn32_per_s) / 2;
            } else if (observer_mode == PNY_OBSERVER_SECANT_LP4 && comm_velocity_count >= COMM_VELOCITY_SAMPLES) {
                comm_velocity_turn32_per_s += (secant_velocity - comm_velocity_turn32_per_s) / 4;
            } else if (observer_mode == PNY_OBSERVER_SECANT_LP8 && comm_velocity_count >= COMM_VELOCITY_SAMPLES) {
                comm_velocity_turn32_per_s += (secant_velocity - comm_velocity_turn32_per_s) / 8;
            } else if (observer_mode == PNY_OBSERVER_SECANT_LP16 && comm_velocity_count >= COMM_VELOCITY_SAMPLES) {
                comm_velocity_turn32_per_s += (secant_velocity - comm_velocity_turn32_per_s) / 16;
            } else {
                comm_velocity_turn32_per_s += (sample_velocity - comm_velocity_turn32_per_s) / 2;
            }
        }
        velocity_turn32_per_s = comm_velocity_turn32_per_s;
        comm_scheduler.position_tick = sample_tick;
    }

    uint16_t control_tick = sched_now_us();
    if ((uint16_t)(control_tick - comm_scheduler.position_tick) > SENSOR_STALE_US) {
        comm_stop_from_isr();
        return;
    }

    vel_counter++;
    if (vel_counter >= VEL_UPDATE_SAMPLES) {
        vel_counter = 0;
        capture_tick_1ms();
    }

    comm_control_tick(control_tick);
    if (have_sample || (TIM_DIER(TIM21) & TIM_DIER_CC2IE) == 0u) {
        comm_scheduler_schedule_sector_event(control_tick, current_direction);
    }
}

static void comm_sector_tick(uint16_t scheduled_tick)
{
    uint16_t now = sched_now_us();
    if ((int16_t)(now - scheduled_tick) < 0) {
        now = scheduled_tick;
    }
    if ((uint16_t)(now - comm_scheduler.position_tick) > SENSOR_STALE_US) {
        comm_stop_from_isr();
        return;
    }

    int8_t direction = current_direction;
    if (direction == 0 || current_duty == 0) {
        TIM_DIER(TIM21) &= ~TIM_DIER_CC2IE;
        return;
    }

    uint8_t sector = comm_scheduler_schedule_sector_event(now, direction);
    if (sector > 5u) {
        return;
    }
    if (sector != comm_scheduler.sector) {
        set_hall_outputs_raw(sector);
        comm_scheduler.sector = sector;
    }
    if (current_duty != comm_scheduler.last_duty) {
        apply_pwm_duty(current_duty);
        comm_scheduler.last_duty = current_duty;
    }
}

void tim21_isr(void)
{
    uint16_t start = sched_now_us();
    uint32_t flags = TIM_SR(TIM21) & TIM_DIER(TIM21);

    if ((flags & TIM_SR_CC2IF) != 0u) {
        uint16_t scheduled_tick = comm_scheduler.next_comm_tick;
        timer_clear_flag(TIM21, TIM_SR_CC2IF);
        comm_sector_tick(scheduled_tick);
    }
    if ((flags & TIM_SR_CC1IF) != 0u) {
        uint16_t scheduled_tick = comm_scheduler.next_sensor_tick;
        timer_clear_flag(TIM21, TIM_SR_CC1IF);
        comm_sensor_tick(scheduled_tick);
    }
    if ((TIM_SR(TIM21) & TIM_SR_UIF) != 0u) {
        timer_clear_flag(TIM21, TIM_SR_UIF);
    }

    isr_duration_us = (uint16_t)(sched_now_us() - start);
    if (isr_duration_us > isr_max_us) {
        isr_max_us = isr_duration_us;
    }
    if ((TIM_SR(TIM21) & TIM_SR_CC1OF) != 0u) {
        isr_overrun_count++;
        timer_clear_flag(TIM21, TIM_SR_CC1OF);
    }
    if ((TIM_SR(TIM21) & TIM_SR_CC2OF) != 0u) {
        isr_overrun_count++;
        timer_clear_flag(TIM21, TIM_SR_CC2OF);
    }
}

void i2c1_isr(void)
{
    tmag5273_i2c1_isr();
}

/* UART protocol */
static bool uart_quiet_active(uint32_t now_ms)
{
    return uart_quiet_until_ms != 0u && (int32_t)(now_ms - uart_quiet_until_ms) < 0;
}

static void send_status_response(uint8_t cmd, uint8_t result)
{
    pny_status_payload_t payload;
    fill_status_payload(&payload, result);
    send_frame(cmd, &payload, sizeof(payload));
}

static void handle_get_status(void)
{
    refresh_status_sensor();
    send_status_response(PNY_CMD_GET_STATUS, PNY_RESULT_OK);
}

static void handle_zero_position(const uint8_t *payload, uint8_t len)
{
    (void)payload;

    if (len != 0u) {
        send_status_response(PNY_CMD_ZERO_POSITION, PNY_RESULT_BAD_ARG);
        return;
    }
    if (current_mode == PNY_MODE_CAL) {
        send_status_response(PNY_CMD_ZERO_POSITION, PNY_RESULT_BAD_STATE);
        return;
    }

    control_disable();
    run_stop();
    refresh_status_sensor();
    position_zero_turn32 = absolute_position_turn32;
    send_status_response(PNY_CMD_ZERO_POSITION, PNY_RESULT_OK);
}

static void handle_stop(const uint8_t *payload, uint8_t len)
{
    (void)payload;

    if (len != 0u) {
        send_status_response(PNY_CMD_STOP, PNY_RESULT_BAD_ARG);
        return;
    }
    control_disable();
    run_stop();
    current_duty = 0;
    current_direction = 0;
    send_status_response(PNY_CMD_STOP, PNY_RESULT_OK);
}

static uint8_t apply_duty_payload(const uint8_t *payload, uint8_t len)
{
    if (len != 2u) {
        return PNY_RESULT_BAD_ARG;
    }

    int16_t duty;
    memcpy(&duty, payload, sizeof(duty));

    control_set_duty(duty, control_clip);
    report_speed = true;
    return control_apply();
}

static void handle_set_duty(const uint8_t *payload, uint8_t len)
{
    send_status_response(PNY_CMD_SET_DUTY, apply_duty_payload(payload, len));
}

static void handle_set_position(const uint8_t *payload, uint8_t len)
{
    if (len != 4u) {
        send_status_response(PNY_CMD_SET_POSITION, PNY_RESULT_BAD_ARG);
        return;
    }

    int32_t position_turn32;
    memcpy(&position_turn32, payload, sizeof(position_turn32));

    target_position_turn32 = position_turn32;
    target_position_set = true;
    send_status_response(PNY_CMD_SET_POSITION, control_apply());
}

static void handle_send_position(const uint8_t *payload, uint8_t len)
{
    if (len != 4u) {
        return;
    }

    int32_t position_turn32;
    memcpy(&position_turn32, payload, sizeof(position_turn32));

    target_position_turn32 = position_turn32;
    target_position_set = true;
    if (current_mode != PNY_MODE_RUN) {
        control_apply();
    }
}

static void handle_send_duty(const uint8_t *payload, uint8_t len)
{
    if (len == 2u) {
        (void)apply_duty_payload(payload, len);
    }
}

static void handle_set_velocity(const uint8_t *payload, uint8_t len)
{
    if (len != 4u) {
        send_status_response(PNY_CMD_SET_VELOCITY, PNY_RESULT_BAD_ARG);
        return;
    }

    memcpy((void *)&target_velocity_turn32_per_s, payload, sizeof(target_velocity_turn32_per_s));
    send_status_response(PNY_CMD_SET_VELOCITY, control_apply());
}

static void handle_set_control(const uint8_t *payload, uint8_t len)
{
    if (len != sizeof(pny_control_payload_t)) {
        send_status_response(PNY_CMD_SET_CONTROL, PNY_RESULT_BAD_ARG);
        return;
    }

    pny_control_payload_t control;
    memcpy(&control, payload, sizeof(control));
    if (control.clip < 0 || control.clip > DUTY_LIMIT ||
        control.kf < -DUTY_LIMIT || control.kf > DUTY_LIMIT) {
        send_status_response(PNY_CMD_SET_CONTROL, PNY_RESULT_RANGE);
        return;
    }

    nvic_disable_irq(NVIC_TIM21_IRQ);
    control_kp_q8 = control.kp_q8;
    control_kd_q8 = control.kd_q8;
    control_kv_q8 = control.kv_q8;
    control_kf = control.kf;
    control_clip = control.clip;
    nvic_enable_irq(NVIC_TIM21_IRQ);
    send_status_response(PNY_CMD_SET_CONTROL, control_apply());
}

static void handle_set_advance(const uint8_t *payload, uint8_t len)
{
    if (len != 2u) {
        send_status_response(PNY_CMD_SET_ADVANCE, PNY_RESULT_BAD_ARG);
        return;
    }

    int16_t advance_deg;
    memcpy(&advance_deg, payload, sizeof(advance_deg));
    if (advance_deg < ADVANCE_MIN_DEG || advance_deg > ADVANCE_MAX_DEG) {
        send_status_response(PNY_CMD_SET_ADVANCE, PNY_RESULT_RANGE);
        return;
    }

    current_lead_turn16 = lead_deg_to_turn16(advance_deg);
    send_status_response(PNY_CMD_SET_ADVANCE, PNY_RESULT_OK);
}

static void handle_set_quiet(const uint8_t *payload, uint8_t len)
{
    uint16_t hold_ms;
    uint8_t result;

    if (len != 2u) {
        result = PNY_RESULT_BAD_ARG;
        send_frame(PNY_CMD_SET_QUIET, &result, 1u);
        return;
    }

    memcpy(&hold_ms, payload, sizeof(hold_ms));
    result = PNY_RESULT_OK;
    send_frame(PNY_CMD_SET_QUIET, &result, 1u);
    while ((USART_ISR(USART2) & USART_ISR_TC) == 0u) {
    }

    if (hold_ms == 0u) {
        uart_quiet_until_ms = 0u;
        return;
    }
    uart_quiet_until_ms = system_millis + hold_ms;
}

static void handle_cal_start(const uint8_t *payload, uint8_t len)
{
    pny_cal_start_payload_t out;
    uint8_t result = PNY_RESULT_BAD_ARG;

    if (len == 1u) {
        result = calibration_start(payload[0]);
    }

    out.result = result;
    out.total_points = PNY_CAL_POINTS_PER_SWEEP;
    send_frame(PNY_CMD_CAL, &out, sizeof(out));
}

static void handle_cal_status(void)
{
    pny_cal_status_payload_t payload;
    payload.result = PNY_RESULT_OK;
    payload.active = cal_state.active ? 1u : 0u;
    payload.next_index = cal_state.index;
    payload.total_points = cal_state.total_points;
    send_frame(PNY_CMD_CAL, &payload, sizeof(payload));
}

static void handle_cal_read_point(const uint8_t *payload, uint8_t len)
{
    pny_cal_point_payload_t out;
    memset(&out, 0, sizeof(out));

    if (len != 1u) {
        out.result = PNY_RESULT_BAD_ARG;
        send_frame(PNY_CMD_CAL, &out, sizeof(out));
        return;
    }

    uint8_t index = payload[0];
    if (index >= cal_state.total_points || (cal_state.active && index >= cal_state.index)) {
        out.result = PNY_RESULT_RANGE;
        send_frame(PNY_CMD_CAL, &out, sizeof(out));
        return;
    }

    out = cal_state.points[index];
    send_frame(PNY_CMD_CAL, &out, sizeof(out));
}

static void handle_cal_write_blob(const uint8_t *payload, uint8_t len)
{
    pny_cal_write_payload_t out;
    out.result = PNY_RESULT_OK;
    out.next_offset = 0;

    if (current_mode != PNY_MODE_IDLE) {
        out.result = PNY_RESULT_BAD_STATE;
        send_frame(PNY_CMD_CAL, &out, sizeof(out));
        return;
    }
    if (len < 3u) {
        out.result = PNY_RESULT_BAD_ARG;
        send_frame(PNY_CMD_CAL, &out, sizeof(out));
        return;
    }

    uint16_t offset = (uint16_t)(payload[0] | ((uint16_t)payload[1] << 8));
    uint8_t chunk_len = (uint8_t)(len - 2u);
    bool ok = pennyesc_calibration_write_chunk(offset, &payload[2], chunk_len);
    flash_fault = !ok;
    out.result = ok ? PNY_RESULT_OK : PNY_RESULT_FLASH;
    out.next_offset = ok ? (uint16_t)(offset + chunk_len) : offset;
    send_frame(PNY_CMD_CAL, &out, sizeof(out));
}

static void handle_cal_commit(void)
{
    pny_cal_commit_payload_t out;
    bool ok = (current_mode == PNY_MODE_IDLE) && pennyesc_calibration_commit();
    flash_fault = !ok;
    out.result = ok ? PNY_RESULT_OK : PNY_RESULT_FLASH;
    out.valid = ok ? 1u : 0u;
    send_frame(PNY_CMD_CAL, &out, sizeof(out));
    if (ok) {
        pending_reset_ms = system_millis + 20u;
    }
}

static void handle_cal_clear(void)
{
    pny_cal_commit_payload_t out;
    bool ok = (current_mode == PNY_MODE_IDLE) && pennyesc_calibration_clear_flash();
    flash_fault = !ok;
    if (ok) {
        position_initialized = false;
        current_angle_turn16 = 0;
    }
    out.result = ok ? PNY_RESULT_OK : PNY_RESULT_FLASH;
    out.valid = 0u;
    send_frame(PNY_CMD_CAL, &out, sizeof(out));
}

static void handle_cal_info(void)
{
    pny_cal_info_payload_t out;
    const pennyesc_calibration_blob_t *blob = pennyesc_calibration_active();
    out.result = PNY_RESULT_OK;
    out.valid = blob ? 1u : 0u;
    out.blob_size = blob ? blob->size : 0u;
    out.blob_crc32 = blob ? blob->crc32 : 0u;
    out.commutation_alignment_turn16 = blob ? blob->commutation_alignment_turn16 : 0u;
    send_frame(PNY_CMD_CAL, &out, sizeof(out));
}

static void fill_capture_status(pny_capture_status_payload_t *out, uint8_t result)
{
    out->subcmd = PNY_DEBUG_CAPTURE_STATUS;
    out->result = result;
    out->active = capture_state.active ? 1u : 0u;
    out->done = capture_state.done ? 1u : 0u;
    out->sample_hz = capture_state.sample_hz;
    out->duration_ms = capture_state.duration_ms;
    out->elapsed_ms = capture_state.elapsed_ms;
    out->sample_count = capture_state.sample_count;
    out->missed_count = capture_state.missed_count;
    out->mct_fault_count = mct_fault_count;
}

static void send_capture_status(uint8_t result)
{
    pny_capture_status_payload_t out;
    fill_capture_status(&out, result);
    send_frame(PNY_CMD_DEBUG, &out, sizeof(out));
}

static void handle_capture_start(const uint8_t *payload, uint8_t len)
{
    if (len != sizeof(pny_capture_start_payload_t)) {
        send_capture_status(PNY_RESULT_BAD_ARG);
        return;
    }

    pny_capture_start_payload_t in;
    memcpy(&in, payload, sizeof(in));
    if (in.duration_ms == 0u || in.duration_ms > CAPTURE_MAX_DURATION_MS ||
        (in.sample_hz != 250u && in.sample_hz != 500u && in.sample_hz != 1000u) ||
        in.advance_deg < ADVANCE_MIN_DEG || in.advance_deg > ADVANCE_MAX_DEG ||
        in.duty > DUTY_LIMIT || in.duty < -DUTY_LIMIT) {
        send_capture_status(PNY_RESULT_RANGE);
        return;
    }

    if (current_mode == PNY_MODE_CAL) {
        send_capture_status(PNY_RESULT_BUSY);
        return;
    }
    uint8_t result = run_ready();
    if (result != PNY_RESULT_OK) {
        send_capture_status(result);
        return;
    }

    nvic_disable_irq(NVIC_TIM21_IRQ);
    memset(&capture_state, 0, sizeof(capture_state));
    capture_state.sample_hz = in.sample_hz;
    capture_state.duration_ms = in.duration_ms;
    capture_state.sample_period_ms = (uint16_t)(1000u / in.sample_hz);
    capture_state.active = true;

    current_lead_turn16 = lead_deg_to_turn16(in.advance_deg);
    control_set_duty(in.duty, DUTY_LIMIT);

    nvic_enable_irq(NVIC_TIM21_IRQ);
    if (current_mode != PNY_MODE_RUN) {
        run_begin();
        if (current_mode != PNY_MODE_RUN) {
            capture_state.active = false;
            capture_state.done = true;
            control_disable();
            send_capture_status(PNY_RESULT_BAD_STATE);
            return;
        }
    }

    send_capture_status(PNY_RESULT_OK);
}

static void handle_capture_read(const uint8_t *payload, uint8_t len)
{
    pny_capture_read_payload_t out;
    memset(&out, 0, sizeof(out));
    out.subcmd = PNY_DEBUG_CAPTURE_READ;

    if (len != 4u || payload[0] != PNY_DEBUG_CAPTURE_READ || payload[3] > PNY_CAPTURE_READ_MAX_SAMPLES) {
        out.result = PNY_RESULT_BAD_ARG;
        send_frame(PNY_CMD_DEBUG, &out, 5u);
        return;
    }
    if (capture_state.active) {
        out.result = PNY_RESULT_BUSY;
        send_frame(PNY_CMD_DEBUG, &out, 5u);
        return;
    }

    uint16_t offset;
    memcpy(&offset, &payload[1], sizeof(offset));
    if (offset > capture_state.sample_count) {
        out.result = PNY_RESULT_RANGE;
        send_frame(PNY_CMD_DEBUG, &out, 5u);
        return;
    }

    uint8_t want = payload[3];
    uint16_t available = (uint16_t)(capture_state.sample_count - offset);
    if (want > available) {
        want = (uint8_t)available;
    }

    out.result = PNY_RESULT_OK;
    out.offset = offset;
    out.count = want;
    for (uint8_t i = 0; i < want; i++) {
        out.samples[i].angle_turn16 = capture_state.samples[offset + i].angle_turn16;
        out.samples[i].rpm = capture_state.samples[offset + i].rpm;
    }
    send_frame(PNY_CMD_DEBUG, &out, (uint8_t)(5u + (want * sizeof(pny_capture_sample_t))));
}

static void handle_set_observer(const uint8_t *payload, uint8_t len)
{
    if (len != sizeof(pny_observer_payload_t)) {
        send_capture_status(PNY_RESULT_BAD_ARG);
        return;
    }

    pny_observer_payload_t in;
    memcpy(&in, payload, sizeof(in));
    if (in.lead_us < OBSERVER_LEAD_MIN_US || in.lead_us > OBSERVER_LEAD_MAX_US || in.mode > PNY_OBSERVER_MAX) {
        send_capture_status(PNY_RESULT_RANGE);
        return;
    }

    observer_lead_us = in.lead_us;
    observer_mode = in.mode;
    observer_state.position_turn32 = absolute_position_turn32;
    send_capture_status(PNY_RESULT_OK);
}

static void handle_cal(const uint8_t *payload, uint8_t len)
{
    if (len == 0u) {
        pny_cal_status_payload_t out = {PNY_RESULT_BAD_ARG, 0, 0, 0};
        send_frame(PNY_CMD_CAL, &out, sizeof(out));
        return;
    }

    switch (payload[0]) {
    case PNY_CAL_START:
        handle_cal_start(&payload[1], (uint8_t)(len - 1u));
        break;
    case PNY_CAL_STATUS:
        handle_cal_status();
        break;
    case PNY_CAL_READ_POINT:
        handle_cal_read_point(&payload[1], (uint8_t)(len - 1u));
        break;
    case PNY_CAL_WRITE_BLOB:
        handle_cal_write_blob(&payload[1], (uint8_t)(len - 1u));
        break;
    case PNY_CAL_COMMIT:
        handle_cal_commit();
        break;
    case PNY_CAL_CLEAR:
        handle_cal_clear();
        break;
    case PNY_CAL_INFO:
        handle_cal_info();
        break;
    default: {
        pny_cal_status_payload_t out = {PNY_RESULT_BAD_ARG, 0, 0, 0};
        send_frame(PNY_CMD_CAL, &out, sizeof(out));
        break;
    }
    }
}

static void handle_debug(const uint8_t *payload, uint8_t len)
{
    if (len == 0u) {
        send_capture_status(PNY_RESULT_BAD_ARG);
        return;
    }

    switch (payload[0]) {
    case PNY_DEBUG_CAPTURE_START:
        handle_capture_start(payload, len);
        break;
    case PNY_DEBUG_CAPTURE_STATUS:
        send_capture_status(PNY_RESULT_OK);
        break;
    case PNY_DEBUG_CAPTURE_READ:
        handle_capture_read(payload, len);
        break;
    case PNY_DEBUG_SET_OBSERVER:
        handle_set_observer(payload, len);
        break;
    default:
        send_capture_status(PNY_RESULT_BAD_ARG);
        break;
    }
}

static void process_frame(uint8_t header, const uint8_t *payload, uint8_t len)
{
    uint8_t address = header >> 4;
    uint8_t cmd = header & 0x0Fu;

    if (address != ESC_ADDRESS) {
        return;
    }
    uart_last_frame_ms = system_millis;

    switch (cmd) {
    case PNY_CMD_GET_STATUS:
        handle_get_status();
        break;
    case PNY_CMD_GET_POS_VEL:
    {
        pny_pos_vel_payload_t out;
        out.position_turn32 = position_from_zero(absolute_position_turn32);
        out.velocity_turn32_per_s = velocity_turn32_per_s;
        send_frame(PNY_CMD_GET_POS_VEL, &out, sizeof(out));
        break;
    }
    case PNY_CMD_ZERO_POSITION:
        handle_zero_position(payload, len);
        break;
    case PNY_CMD_STOP:
        handle_stop(payload, len);
        break;
    case PNY_CMD_SET_DUTY:
        handle_set_duty(payload, len);
        break;
    case PNY_CMD_SET_POSITION:
        handle_set_position(payload, len);
        break;
    case PNY_CMD_SEND_POSITION:
        handle_send_position(payload, len);
        break;
    case PNY_CMD_SEND_DUTY:
        handle_send_duty(payload, len);
        break;
    case PNY_CMD_SET_VELOCITY:
        handle_set_velocity(payload, len);
        break;
    case PNY_CMD_SET_CONTROL:
        handle_set_control(payload, len);
        break;
    case PNY_CMD_SET_ADVANCE:
        handle_set_advance(payload, len);
        break;
    case PNY_CMD_SET_QUIET:
        handle_set_quiet(payload, len);
        break;
    case PNY_CMD_CAL:
        handle_cal(payload, len);
        break;
    case PNY_CMD_DEBUG:
        handle_debug(payload, len);
        break;
    default:
        break;
    }
}

static void uart_poll(void)
{
    uint16_t head = (uint16_t)((UART_DMA_RX_BUF_SIZE - dma_get_number_of_data(DMA1, DMA_CHANNEL5)) & UART_DMA_RX_BUF_MASK);

    while (uart_dma_tail != head) {
        uint8_t byte = uart_dma_rx[uart_dma_tail];
        uart_dma_tail = (uint16_t)((uart_dma_tail + 1u) & UART_DMA_RX_BUF_MASK);

        if (uart_quiet_active(system_millis)) {
            pny_frame_parser_reset(&frame_parser);
            continue;
        }

        if (pennyesc_uart_update_feed_byte(
                byte,
                ESC_ADDRESS,
                system_millis,
                fill_update_status,
                prepare_update_boot)) {
            pny_frame_parser_reset(&frame_parser);
            continue;
        }

        const uint8_t *frame;
        uint8_t frame_len;
        if (pny_frame_parser_push(&frame_parser, byte, system_millis, UART_FRAME_TIMEOUT_MS, &frame, &frame_len)) {
            process_frame(frame[1], &frame[3], frame[2]);
        }
    }

    if ((USART_ISR(USART2) & USART_ISR_ORE) != 0u) {
        USART_ICR(USART2) = USART_ICR_ORECF;
        uart_overrun_errors++;
    }

}

int main(void)
{
    clock_setup();
    systick_setup();
    __asm__("cpsie i");
    usart2_setup(PNY_UART_UPDATE_BOOT_BAUD);
    pennyesc_uart_update_boot_window(&system_millis, ESC_ADDRESS);
    usart2_setup(pennyesc_uart_update_app_baud(UART_APP_BAUD));
    i2c1_setup();
    tim2_pwm_setup();
    tim21_setup();
    motor_gpio_setup();

    mct8316z_init();
    delay_ms(5);
    mct_apply_config();

    pennyesc_calibration_load();
    flash_fault = false;

    if (sensor_init_full_mode()) {
        refresh_sensor_xyz(pennyesc_calibration_valid());
    }

    while (1) {
        uart_poll();
        if (current_mode == PNY_MODE_CAL) {
            calibration_poll();
        }

        mct_idle_check();
        control_restart_poll();
        pennyesc_uart_update_poll(system_millis);
        if (pending_reset_ms != 0u && (int32_t)(system_millis - pending_reset_ms) >= 0) {
            scb_reset_system();
        }
    }
}
