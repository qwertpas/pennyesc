#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "mct8316z.h"
#include "pennyesc_boot.h"
#include "pennyesc_calibration.h"
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
#define DUTY_LIMIT PWM_PERIOD
#define DEFAULT_POSITION_DUTY 150
#define POSITION_DEADBAND_CRAD 6
#define ADVANCE_DEG 40
#define ADVANCE_MIN_DEG -180
#define ADVANCE_MAX_DEG 180

#define ISR_FREQ_HZ 4000
#define ISR_PERIOD_US (1000000u / ISR_FREQ_HZ)
#define ISR_TIMER_PERIOD ((1000000u / ISR_FREQ_HZ) - 1u)
#define VEL_UPDATE_SAMPLES (ISR_FREQ_HZ / 1000u)

#define TMAG_RUN_DELAY_US 113u
#define OBS_ALPHA_NUM 3
#define OBS_ALPHA_SHIFT 3
#define OBS_BETA_SHIFT 4

#define CAL_DUTY 100
#define CAL_SETTLE_MS 180u
#define CAL_SAMPLE_INTERVAL_MS 2u
#define CAL_SAMPLE_COUNT 16u
#define SENSOR_IDLE_POLL_MS 5u
#define UART_FRAME_TIMEOUT_MS 10u
#define UART_APP_BAUD 230400u

#define FRAME_BUF_SIZE (PNY_FRAME_MAX_PAYLOAD + 4u)

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

static volatile uint32_t system_millis;

static volatile int16_t sensor_x;
static volatile int16_t sensor_y;
static volatile int16_t sensor_z;
static volatile uint16_t current_angle_turn16;
static volatile int32_t absolute_position_turn32;
static volatile int32_t last_position_turn32;
static volatile int32_t velocity_turn32_per_s;

static volatile bool position_initialized;
static volatile uint16_t last_angle_turn16;
static volatile bool sensor_ready;
static volatile bool flash_fault;

static volatile uint8_t current_mode = PNY_MODE_IDLE;
static volatile bool target_position_set;
static volatile int32_t target_position_turn32;
static volatile int16_t commanded_duty;
static volatile int16_t current_duty;
static volatile int8_t current_direction;
static volatile int16_t current_advance_deg = ADVANCE_DEG;
static volatile uint32_t pending_reset_ms;

static volatile uint32_t isr_duration_us;
static volatile uint32_t isr_max_us;
static volatile uint32_t isr_count;
static volatile uint32_t isr_overrun_count;
static volatile uint32_t uart_crc_errors;
static volatile uint32_t uart_bytes_rx;
static volatile uint32_t uart_overrun_errors;
static volatile uint32_t looptime_us;
static volatile int32_t debug_position;
static volatile int32_t debug_velocity;
static volatile int debug_comm_step;
static volatile int32_t observer_angle_turn32;
static volatile int32_t observer_velocity_turn32_per_s;
static volatile bool observer_ready;

static calibration_state_t cal_state;

static uint8_t frame_buf[FRAME_BUF_SIZE];
static uint8_t frame_idx;
static uint8_t frame_expected;
static uint32_t frame_last_byte_ms;
static volatile uint32_t uart_quiet_until_ms;
static uint32_t last_idle_sensor_ms;
static bool sensor_run_mode;

static void stop_motor_outputs(void);

static const uint8_t hall_pattern[6][3] = {
    {1, 1, 0},
    {1, 0, 0},
    {1, 0, 1},
    {0, 0, 1},
    {0, 1, 1},
    {0, 1, 0},
};

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

static uint32_t get_time_us(void)
{
    uint32_t millis = system_millis;
    uint32_t ticks_elapsed = 31999u - systick_get_value();
    return (millis * 1000u) + (ticks_elapsed >> 5);
}

static uint8_t crc8_calculate(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0u;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
        }
    }

    return crc;
}

static int32_t div_round_i64(int64_t value, int32_t denom)
{
    if (value >= 0) {
        return (int32_t)((value + (denom / 2)) / denom);
    }
    return -(int32_t)(((-value) + (denom / 2)) / denom);
}

static int32_t turn32_to_crad(int32_t turn32)
{
    return div_round_i64((int64_t)turn32 * 628, 65536);
}

static int32_t crad_to_turn32(int32_t crad)
{
    return div_round_i64((int64_t)crad * 65536, 628);
}

static void observer_invalidate(void)
{
    observer_ready = false;
    observer_angle_turn32 = 0;
    observer_velocity_turn32_per_s = 0;
}

static void observer_predict_tick(void)
{
    if (!observer_ready) {
        return;
    }
    observer_angle_turn32 += div_round_i64((int64_t)observer_velocity_turn32_per_s * ISR_PERIOD_US, 1000000);
}

static void observer_update(uint16_t angle_turn16)
{
    if (!observer_ready) {
        observer_angle_turn32 = angle_turn16;
        observer_velocity_turn32_per_s = 0;
        observer_ready = true;
        return;
    }

    int32_t delayed =
        observer_angle_turn32 - div_round_i64((int64_t)observer_velocity_turn32_per_s * TMAG_RUN_DELAY_US, 1000000);
    int32_t error = (int16_t)(angle_turn16 - (uint16_t)delayed);

    observer_angle_turn32 += div_round_i64((int64_t)error * OBS_ALPHA_NUM, 1 << OBS_ALPHA_SHIFT);
    observer_velocity_turn32_per_s += div_round_i64((int64_t)error * ISR_FREQ_HZ, 1 << OBS_BETA_SHIFT);
}

static uint16_t observer_angle_turn16(void)
{
    if (!observer_ready) {
        return current_angle_turn16;
    }
    return (uint16_t)observer_angle_turn32;
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

static uint8_t frame_header(uint8_t cmd)
{
    return (uint8_t)((ESC_ADDRESS << 4) | (cmd & 0x0Fu));
}

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

static uint8_t current_flags(void)
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
    if (target_position_set && current_duty == 0) {
        flags |= PNY_FLAG_POSITION_REACHED;
    }
    if (current_faults() != 0u) {
        flags |= PNY_FLAG_FAULT;
    }
    return flags;
}

static void send_frame(uint8_t cmd, const void *payload, uint8_t payload_len)
{
    uint8_t tx[FRAME_BUF_SIZE];

    tx[0] = PNY_FRAME_START;
    tx[1] = frame_header(cmd);
    tx[2] = payload_len;
    if (payload_len != 0u) {
        memcpy(&tx[3], payload, payload_len);
    }
    tx[3 + payload_len] = crc8_calculate(tx, (uint8_t)(3u + payload_len));

    for (uint8_t i = 0; i < (uint8_t)(4u + payload_len); i++) {
        usart_send_blocking(USART2, tx[i]);
    }
}

static void usart2_setup(uint32_t baud)
{
    usart_disable(USART2);
    gpio_set_af(GPIOA, GPIO_AF4, GPIO9 | GPIO10);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO9 | GPIO10);

    usart_set_baudrate(USART2, baud);
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
    I2C_TIMINGR(I2C1) = 0x00100109;
    i2c_peripheral_enable(I2C1);
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
    timer_set_period(TIM21, ISR_TIMER_PERIOD);
    timer_enable_irq(TIM21, TIM_DIER_UIE);
    nvic_enable_irq(NVIC_TIM21_IRQ);
    nvic_set_priority(NVIC_TIM21_IRQ, 0);
    timer_disable_counter(TIM21);
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

static void clear_hall_outputs(void)
{
    gpio_clear(HALLA_PORT, HALLA_PIN);
    gpio_clear(HALLB_PORT, HALLB_PIN);
    gpio_clear(HALLC_PORT, HALLC_PIN);
}

static void set_hall_outputs(int step)
{
    step = ((step % 6) + 6) % 6;
    const uint8_t *pattern = hall_pattern[step];
    if (pattern[0]) {
        gpio_set(HALLA_PORT, HALLA_PIN);
    } else {
        gpio_clear(HALLA_PORT, HALLA_PIN);
    }
    if (pattern[1]) {
        gpio_set(HALLB_PORT, HALLB_PIN);
    } else {
        gpio_clear(HALLB_PORT, HALLB_PIN);
    }
    if (pattern[2]) {
        gpio_set(HALLC_PORT, HALLC_PIN);
    } else {
        gpio_clear(HALLC_PORT, HALLC_PIN);
    }
}

static void apply_pwm_duty(int16_t duty)
{
    uint16_t pwm = (duty < 0) ? (uint16_t)(-duty) : (uint16_t)duty;
    if (pwm > DUTY_LIMIT) {
        pwm = DUTY_LIMIT;
    }
    timer_set_oc_value(TIM2, TIM_OC4, pwm);
}

static void update_position_from_angle(uint16_t angle_turn16)
{
    if (!position_initialized) {
        last_angle_turn16 = angle_turn16;
        last_position_turn32 = absolute_position_turn32;
        velocity_turn32_per_s = 0;
        position_initialized = true;
        return;
    }

    int16_t delta = (int16_t)(angle_turn16 - last_angle_turn16);
    absolute_position_turn32 += delta;
    last_angle_turn16 = angle_turn16;
}

static bool sensor_set_full_mode(void)
{
    if (!sensor_run_mode) {
        return true;
    }
    if (!tmag5273_set_mode(TMAG5273_MODE_FULL)) {
        return false;
    }
    sensor_run_mode = false;
    return true;
}

static bool sensor_set_run_mode(void)
{
    if (sensor_run_mode) {
        return true;
    }
    if (!tmag5273_set_mode(TMAG5273_MODE_RUN)) {
        return false;
    }
    if (!tmag5273_prime_run_mode()) {
        return false;
    }
    delay_ms(1u);
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

static void refresh_sensor_xy(bool update_position)
{
    int16_t x;
    int16_t y;

    if (!tmag5273_read_xy_fast_triggered(&x, &y)) {
        sensor_ready = false;
        current_angle_turn16 = 0;
        target_position_set = false;
        commanded_duty = 0;
        timer_disable_counter(TIM21);
        current_mode = PNY_MODE_IDLE;
        observer_invalidate();
        stop_motor_outputs();
        return;
    }
    sensor_ready = true;
    sensor_x = x;
    sensor_y = y;
    sensor_z = 0;

    if (pennyesc_calibration_valid()) {
        uint16_t angle = pennyesc_calibration_angle_turn16(x, y);
        current_angle_turn16 = angle;
        if (update_position) {
            update_position_from_angle(angle);
        }
        observer_update(angle);
    } else {
        current_angle_turn16 = 0;
        observer_invalidate();
    }
}

static void stop_motor_outputs(void)
{
    apply_pwm_duty(0);
    clear_hall_outputs();
    current_duty = 0;
    current_direction = 0;
}

static uint16_t advance_turn16(void)
{
    int32_t advance = current_advance_deg;
    if (advance < 0) {
        advance += 360;
    }
    return (uint16_t)(((uint32_t)advance * 65536u + 180u) / 360u);
}

static int mechanical_angle_to_step(uint16_t angle_turn16, int direction)
{
    int32_t electrical = (int32_t)((uint32_t)angle_turn16 * POLE_PAIRS);
    if (direction < 0) {
        electrical = -electrical;
    }
    if (direction != 0) {
        electrical += advance_turn16();
    }
    return (int)(((uint16_t)electrical * 6u) >> 16);
}

static void run_stop(void)
{
    timer_disable_counter(TIM21);
    observer_invalidate();
    if (current_mode == PNY_MODE_RUN) {
        current_mode = PNY_MODE_IDLE;
    }
    stop_motor_outputs();
}

static uint8_t run_ready(void)
{
    if (!sensor_ready) {
        return PNY_RESULT_BAD_STATE;
    }
    if (!pennyesc_calibration_valid()) {
        return PNY_RESULT_NOT_CALIBRATED;
    }
    if (current_mode == PNY_MODE_CAL) {
        return PNY_RESULT_BUSY;
    }
    return PNY_RESULT_OK;
}

static void run_begin(void)
{
    observer_invalidate();
    if (!sensor_set_full_mode()) {
        sensor_ready = false;
        current_mode = PNY_MODE_IDLE;
        stop_motor_outputs();
        return;
    }
    refresh_sensor_xyz(true);
    if (!sensor_ready) {
        current_mode = PNY_MODE_IDLE;
        stop_motor_outputs();
        return;
    }
    current_mode = PNY_MODE_RUN;
    timer_set_counter(TIM21, 0);
    timer_enable_counter(TIM21);
}

static void fill_status_payload(pny_status_payload_t *payload, uint8_t result)
{
    payload->result = result;
    payload->mode = current_mode;
    payload->flags = current_flags();
    payload->faults = current_faults();
    payload->x = sensor_x;
    payload->y = sensor_y;
    payload->z = sensor_z;
    payload->angle_turn16 = current_angle_turn16;
    payload->position_crad = turn32_to_crad(absolute_position_turn32);
    payload->velocity_crads = turn32_to_crad(velocity_turn32_per_s);
    payload->duty = current_duty;
}

static void fill_update_status(pny_status_payload_t *payload)
{
    fill_status_payload(payload, PNY_RESULT_OK);
}

static uint8_t prepare_update_boot(void)
{
    if (current_mode == PNY_MODE_CAL) {
        return PNY_RESULT_BUSY;
    }

    target_position_set = false;
    commanded_duty = 0;
    run_stop();
    return PNY_RESULT_OK;
}

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
    set_hall_outputs(step % 6u);
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
    if (!sensor_ready) {
        return PNY_RESULT_BAD_STATE;
    }
    if (sweep_dir > 1u) {
        return PNY_RESULT_BAD_ARG;
    }
    if (!sensor_set_full_mode()) {
        sensor_ready = false;
        return PNY_RESULT_BAD_STATE;
    }

    memset(&cal_state, 0, sizeof(cal_state));
    target_position_set = false;
    commanded_duty = 0;
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
        target_position_set = false;
        commanded_duty = 0;
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

void tim21_isr(void)
{
    uint32_t t0 = get_time_us();
    static uint8_t vel_counter;

    timer_clear_flag(TIM21, TIM_SR_UIF);

    refresh_sensor_xyz(true);

    vel_counter++;
    if (vel_counter >= VEL_UPDATE_SAMPLES) {
        velocity_turn32_per_s = (absolute_position_turn32 - last_position_turn32) * 1000;
        last_position_turn32 = absolute_position_turn32;
        vel_counter = 0;
    }

    int16_t duty = commanded_duty;
    int8_t direction = 0;

    if (target_position_set) {
        int32_t error = target_position_turn32 - absolute_position_turn32;
        int32_t deadband = crad_to_turn32(POSITION_DEADBAND_CRAD);
        int16_t drive = commanded_duty;
        if (drive < 0) {
            drive = (int16_t)(-drive);
        }
        if (drive == 0) {
            drive = DEFAULT_POSITION_DUTY;
        }

        if (error > deadband) {
            duty = drive;
            direction = 1;
        } else if (error < -deadband) {
            duty = (int16_t)(-drive);
            direction = -1;
        } else {
            duty = 0;
            direction = 0;
        }
    } else if (duty > 0) {
        direction = 1;
    } else if (duty < 0) {
        direction = -1;
    }

    current_duty = duty;
    current_direction = direction;

    if (duty == 0) {
        stop_motor_outputs();
        if (!target_position_set) {
            timer_disable_counter(TIM21);
            current_mode = PNY_MODE_IDLE;
        }
        debug_comm_step = -1;
    } else {
        apply_pwm_duty(duty);
        debug_comm_step = mechanical_angle_to_step(current_angle_turn16, direction);
        set_hall_outputs(debug_comm_step);
    }

    isr_duration_us = get_time_us() - t0;
    if (isr_duration_us > isr_max_us) {
        isr_max_us = isr_duration_us;
    }
    if (isr_duration_us > ISR_PERIOD_US) {
        isr_overrun_count++;
    }
    isr_count++;

    debug_position = turn32_to_crad(absolute_position_turn32);
    debug_velocity = turn32_to_crad(velocity_turn32_per_s);
}

static void parser_reset(void)
{
    frame_idx = 0;
    frame_expected = 0;
}

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

static void handle_set_duty(const uint8_t *payload, uint8_t len)
{
    if (len != 2u) {
        send_status_response(PNY_CMD_SET_DUTY, PNY_RESULT_BAD_ARG);
        return;
    }

    int16_t duty;
    memcpy(&duty, payload, sizeof(duty));

    int16_t old_commanded_duty = commanded_duty;
    bool old_target_position_set = target_position_set;
    bool was_running = (current_mode == PNY_MODE_RUN);
    target_position_set = false;
    commanded_duty = duty;

    if (duty == 0) {
        current_mode = PNY_MODE_IDLE;
        current_duty = 0;
        current_direction = 0;
        send_status_response(PNY_CMD_SET_DUTY, PNY_RESULT_OK);
        run_stop();
        return;
    }

    uint8_t result = was_running ? PNY_RESULT_OK : run_ready();
    if (result != PNY_RESULT_OK) {
        commanded_duty = old_commanded_duty;
        target_position_set = old_target_position_set;
        send_status_response(PNY_CMD_SET_DUTY, result);
        return;
    }
    current_mode = PNY_MODE_RUN;
    send_status_response(PNY_CMD_SET_DUTY, PNY_RESULT_OK);
    if (!was_running) {
        run_begin();
    }
}

static void handle_set_position(const uint8_t *payload, uint8_t len)
{
    if (len != 4u) {
        send_status_response(PNY_CMD_SET_POSITION, PNY_RESULT_BAD_ARG);
        return;
    }

    int32_t position_crad;
    memcpy(&position_crad, payload, sizeof(position_crad));

    int16_t old_commanded_duty = commanded_duty;
    bool old_target_position_set = target_position_set;
    int32_t old_target_position_turn32 = target_position_turn32;
    bool was_running = (current_mode == PNY_MODE_RUN);
    target_position_turn32 = crad_to_turn32(position_crad);
    target_position_set = true;
    if (commanded_duty == 0) {
        commanded_duty = DEFAULT_POSITION_DUTY;
    }

    uint8_t result = was_running ? PNY_RESULT_OK : run_ready();
    if (result != PNY_RESULT_OK) {
        commanded_duty = old_commanded_duty;
        target_position_set = old_target_position_set;
        target_position_turn32 = old_target_position_turn32;
        send_status_response(PNY_CMD_SET_POSITION, result);
        return;
    }
    current_mode = PNY_MODE_RUN;
    send_status_response(PNY_CMD_SET_POSITION, PNY_RESULT_OK);
    if (!was_running) {
        run_begin();
    }
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

    current_advance_deg = advance_deg;
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
    send_frame(PNY_CMD_CAL_START, &out, sizeof(out));
}

static void handle_cal_status(void)
{
    pny_cal_status_payload_t payload;
    payload.result = PNY_RESULT_OK;
    payload.active = cal_state.active ? 1u : 0u;
    payload.next_index = cal_state.index;
    payload.total_points = cal_state.total_points;
    send_frame(PNY_CMD_CAL_STATUS, &payload, sizeof(payload));
}

static void handle_cal_read_point(const uint8_t *payload, uint8_t len)
{
    pny_cal_point_payload_t out;
    memset(&out, 0, sizeof(out));

    if (len != 1u) {
        out.result = PNY_RESULT_BAD_ARG;
        send_frame(PNY_CMD_CAL_READ_POINT, &out, sizeof(out));
        return;
    }

    uint8_t index = payload[0];
    if (index >= cal_state.total_points || (cal_state.active && index >= cal_state.index)) {
        out.result = PNY_RESULT_RANGE;
        send_frame(PNY_CMD_CAL_READ_POINT, &out, sizeof(out));
        return;
    }

    out = cal_state.points[index];
    send_frame(PNY_CMD_CAL_READ_POINT, &out, sizeof(out));
}

static void handle_cal_write_blob(const uint8_t *payload, uint8_t len)
{
    pny_cal_write_payload_t out;
    out.result = PNY_RESULT_OK;
    out.next_offset = 0;

    if (current_mode != PNY_MODE_IDLE) {
        out.result = PNY_RESULT_BAD_STATE;
        send_frame(PNY_CMD_CAL_WRITE_BLOB, &out, sizeof(out));
        return;
    }
    if (len < 3u) {
        out.result = PNY_RESULT_BAD_ARG;
        send_frame(PNY_CMD_CAL_WRITE_BLOB, &out, sizeof(out));
        return;
    }

    uint16_t offset = (uint16_t)(payload[0] | ((uint16_t)payload[1] << 8));
    uint8_t chunk_len = (uint8_t)(len - 2u);
    bool ok = pennyesc_calibration_write_chunk(offset, &payload[2], chunk_len);
    flash_fault = !ok;
    out.result = ok ? PNY_RESULT_OK : PNY_RESULT_FLASH;
    out.next_offset = ok ? (uint16_t)(offset + chunk_len) : offset;
    send_frame(PNY_CMD_CAL_WRITE_BLOB, &out, sizeof(out));
}

static void handle_cal_commit(void)
{
    pny_cal_commit_payload_t out;
    bool ok = (current_mode == PNY_MODE_IDLE) && pennyesc_calibration_commit();
    flash_fault = !ok;
    out.result = ok ? PNY_RESULT_OK : PNY_RESULT_FLASH;
    out.valid = ok ? 1u : 0u;
    send_frame(PNY_CMD_CAL_COMMIT, &out, sizeof(out));
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
    send_frame(PNY_CMD_CAL_CLEAR, &out, sizeof(out));
}

static void handle_cal_info(void)
{
    pny_cal_info_payload_t out;
    const pennyesc_calibration_blob_t *blob = pennyesc_calibration_active();
    out.result = PNY_RESULT_OK;
    out.valid = blob ? 1u : 0u;
    out.blob_size = blob ? blob->size : 0u;
    out.blob_crc32 = blob ? blob->crc32 : 0u;
    send_frame(PNY_CMD_CAL_INFO, &out, sizeof(out));
}

static void process_frame(uint8_t header, const uint8_t *payload, uint8_t len)
{
    uint8_t address = header >> 4;
    uint8_t cmd = header & 0x0Fu;

    if (address != ESC_ADDRESS) {
        return;
    }

    switch (cmd) {
    case PNY_CMD_SET_DUTY:
        handle_set_duty(payload, len);
        break;
    case PNY_CMD_SET_POSITION:
        handle_set_position(payload, len);
        break;
    case PNY_CMD_SET_ADVANCE:
        handle_set_advance(payload, len);
        break;
    case PNY_CMD_SET_QUIET:
        handle_set_quiet(payload, len);
        break;
    case PNY_CMD_CAL_START:
        handle_cal_start(payload, len);
        break;
    case PNY_CMD_CAL_STATUS:
        handle_cal_status();
        break;
    case PNY_CMD_CAL_READ_POINT:
        handle_cal_read_point(payload, len);
        break;
    case PNY_CMD_CAL_WRITE_BLOB:
        handle_cal_write_blob(payload, len);
        break;
    case PNY_CMD_CAL_COMMIT:
        handle_cal_commit();
        break;
    case PNY_CMD_CAL_CLEAR:
        handle_cal_clear();
        break;
    case PNY_CMD_CAL_INFO:
        handle_cal_info();
        break;
    default:
        break;
    }
}

static void uart_poll(void)
{
    while ((USART_ISR(USART2) & USART_ISR_RXNE) != 0u) {
        uint8_t byte = (uint8_t)USART_RDR(USART2);
        uart_bytes_rx++;
        frame_last_byte_ms = system_millis;

        if (uart_quiet_active(system_millis)) {
            parser_reset();
            continue;
        }

        if (pennyesc_uart_update_feed_byte(
                byte,
                ESC_ADDRESS,
                system_millis,
                fill_update_status,
                prepare_update_boot)) {
            parser_reset();
            continue;
        }

        if (frame_idx == 0u) {
            if (byte != PNY_FRAME_START) {
                continue;
            }
            frame_buf[frame_idx++] = byte;
            continue;
        }

        if (frame_idx < 3u && byte == PNY_FRAME_START) {
            frame_buf[0] = byte;
            frame_idx = 1u;
            frame_expected = 0u;
            continue;
        }

        frame_buf[frame_idx++] = byte;

        if (frame_idx == 3u) {
            if (frame_buf[2] > PNY_FRAME_MAX_PAYLOAD) {
                parser_reset();
                continue;
            }
            frame_expected = (uint8_t)(frame_buf[2] + 4u);
        }

        if (frame_expected != 0u && frame_idx == frame_expected) {
            uint8_t crc = crc8_calculate(frame_buf, (uint8_t)(frame_expected - 1u));
            if (crc == frame_buf[frame_expected - 1u]) {
                process_frame(frame_buf[1], &frame_buf[3], frame_buf[2]);
            } else {
                uart_crc_errors++;
            }
            parser_reset();
            continue;
        }

        if (frame_idx >= FRAME_BUF_SIZE) {
            parser_reset();
        }
    }

    if ((USART_ISR(USART2) & USART_ISR_ORE) != 0u) {
        USART_ICR(USART2) = USART_ICR_ORECF;
        uart_overrun_errors++;
    }

    if (frame_idx != 0u && (system_millis - frame_last_byte_ms) > UART_FRAME_TIMEOUT_MS) {
        parser_reset();
    }
}

static void idle_sensor_poll(void)
{
    if (!sensor_ready || current_mode != PNY_MODE_IDLE) {
        return;
    }
    if (!sensor_set_full_mode()) {
        sensor_ready = false;
        return;
    }
    if ((system_millis - last_idle_sensor_ms) < SENSOR_IDLE_POLL_MS) {
        return;
    }
    last_idle_sensor_ms = system_millis;
    refresh_sensor_xyz(pennyesc_calibration_valid());
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
    mct8316z_set_pwm_mode_async_dig();
    mct8316z_write_reg(0x0A, 0x03);

    sensor_ready = tmag5273_init();
    if (sensor_ready) {
        tmag5273_clear_por();
    }

    pennyesc_calibration_load();
    flash_fault = false;

    if (sensor_ready) {
        refresh_sensor_xyz(false);
        if (pennyesc_calibration_valid()) {
            last_angle_turn16 = current_angle_turn16;
            position_initialized = true;
        }
    }

    uint32_t last_loop_us = 0u;
    while (1) {
        uart_poll();
        if (current_mode == PNY_MODE_CAL) {
            calibration_poll();
        } else {
            idle_sensor_poll();
        }

        pennyesc_uart_update_poll(system_millis);
        if (pending_reset_ms != 0u && (int32_t)(system_millis - pending_reset_ms) >= 0) {
            scb_reset_system();
        }

        uint32_t now_us = get_time_us();
        if (last_loop_us != 0u) {
            looptime_us = now_us - last_loop_us;
        }
        last_loop_us = now_us;
    }
}
