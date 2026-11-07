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
#include "pennyesc_calibration.h"
#include "pennyesc_frame.h"
#include "pennyesc_timing.h"
#include "pennyesc_protocol.h"
#include "tmag5273.h"

#ifndef PNY_ESC_ADDRESS
#define PNY_ESC_ADDRESS 1
#endif

#ifndef PNY_DRIVE_BRUSHED
#define PNY_DRIVE_BRUSHED 0
#endif

/* The web flasher sets the last byte before writing the application image. */
static const volatile uint8_t firmware_address[] = {'P', 'N', 'Y', 'A', 'D', 'D', 'R', PNY_ESC_ADDRESS};
#define ESC_ADDRESS firmware_address[7]

#if PNY_ESC_ADDRESS > 0xF
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
#define BRUSHED_CONTROL_GAIN_LIMIT_Q8 (300 * CONTROL_GAIN_SCALE)
#define CONTROL_TURN32_NUM 25
#define CONTROL_TURN32_SHIFT 26
#define ADVANCE_DEG 90
#define LEAD_DEFAULT_TURN16 ((int32_t)(((int32_t)ADVANCE_DEG * 65536 + 180) / 360))
#define REVERSE_HALL_PHASE_TURN16 32768
#define ADVANCE_MIN_DEG -180
#define ADVANCE_MAX_DEG 180
/* Lead past the estimated magnetic sample instant. Bench, 1104 motor with a
 * drag load, 12k-35k rpm: supply current at a held speed is lowest from
 * -10 to -20 us (most torque per amp); +20 us costs 1-3%, -60 us 6-9%. */
#define OBSERVER_LEAD_US -15
#define OBSERVER_LEAD_MIN_US -1000
#define OBSERVER_LEAD_MAX_US 1000
#ifndef SENSOR_TICK_US
/* Each ~90 us transaction triggers a 75 us XY conversion ~20 us in and reads
 * the previous one. A conversion that completes while the bus talks to the
 * TMAG is lost, so it must land between this read's end and the next start. */
#define SENSOR_TICK_US 125u
#endif
/* Triggered conversion: register byte acked ~20 us after START, X and Y
 * sampled 25 us apart. Stamp at the midpoint. Bench-tune with lead_us. */
#define TRIGGER_SAMPLE_OFFSET_US 45
/* Field weakening trades torque per amp for top speed (unloaded at 35k rpm,
 * +30 us drew 1.15 A against 0.30 A), so it is off unless asked for. */
#define FW_DEFAULT_US 0
#define FW_MAX_DEFAULT_DEG 30
#define OBS_ALPHA_SHIFT 3u
#define OBS_BETA_SHIFT 6u
#define OBS_GAMMA_SHIFT 12u
/* A sample this far from the prediction (about 22 degrees mechanical) is
 * treated as a glitch, unless the previous one was also rejected. */
#define OBS_GATE_TURN32 (4096 << 16)
/* Slip guard: deceleration against the drive above this, while faster than
 * GUARD_MIN_RPM, removes field-weakening advance for GUARD_MS (no effect
 * while fw_us is 0). */
#define GUARD_ACCEL_RPM_S 600000.0
#define GUARD_MIN_RPM 5000.0
#define GUARD_MS 20u
#define TICK_S ((double)SENSOR_TICK_US * 1e-6)
#define RPM_TO_TURN32_PER_TICK(rpm) ((rpm) / 60.0 * TICK_S * 4294967296.0)
#define GUARD_ACCEL_TURN32 ((int32_t)(GUARD_ACCEL_RPM_S / 60.0 * TICK_S * TICK_S * 4294967296.0))
#define GUARD_MIN_TURN32 ((uint32_t)RPM_TO_TURN32_PER_TICK(GUARD_MIN_RPM))
#define GUARD_TICKS ((uint16_t)(GUARD_MS * 1000u / SENSOR_TICK_US))
/* Observer turn32/tick -> turn16/s and turn32/tick^2 -> krpm/s (>> 22). */
#define VEL_TURN16_PER_S_MUL (1000000u / SENSOR_TICK_US)
#define ACCEL_KRPM_MUL ((int32_t)(60e9 / 1024.0 / ((double)SENSOR_TICK_US * SENSOR_TICK_US) + 0.5))
/* Microseconds -> Q8 sensor ticks (after >> 16), and observer velocity ->
 * electrical turn16 per microsecond in Q8 (velocity >> 14 times
 * RATE_Q8_MUL >> 6, then >> 12). */
#define TICK_Q24_PER_US ((int32_t)((16777216u + SENSOR_TICK_US / 2u) / SENSOR_TICK_US))
#define RATE_Q8_MUL ((int64_t)((6.0 * 16777216.0) / SENSOR_TICK_US + 0.5))
#define SECTOR_TURN16_Q16 715827883u
#define SCHED_TIMER_PERIOD 0xffffu
#define SENSOR_STALE_US 10000u
#define SENSOR_START_RETRIES 4u

#if PNY_DRIVE_BRUSHED
#define BRUSHED_FORWARD_STEP 2u
#define BRUSHED_REVERSE_STEP 5u
#define BRUSHED_FRICTION_DUTY 150
#define BRUSHED_FRICTION_RAMP_TURN32 1043
#define BRUSHED_POSITION_HOLD_TURN32 209
#define BRUSHED_VELOCITY_UPDATE_SAMPLES 100u
#endif

#define CAL_DUTY 100
#define CAL_SETTLE_MS 180u
#define CAL_SAMPLE_INTERVAL_MS 2u
#define CAL_SAMPLE_COUNT 16u
#define UART_FRAME_TIMEOUT_MS 10u
#define UART_APP_BAUD 921600u
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

/* Main-context updates of state the PendSV estimator reads. */
#define irq_lock() __asm__ volatile("cpsid i" ::: "memory")
#define irq_unlock() __asm__ volatile("cpsie i" ::: "memory")

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

/* Ring buffer of per-sensor-tick records. It freezes a quarter buffer after a
 * slip-guard or commutation fault, or when duration_ms ends. */
typedef struct {
    volatile bool active;
    volatile bool done;
    volatile bool owns_motor;
    volatile uint8_t decimation;
    volatile uint8_t decim_count;
    volatile uint16_t duration_ms;
    volatile uint16_t post_trigger;
    volatile uint16_t trigger_index;
    volatile uint32_t start_ms;
    volatile uint16_t sample_count;
    volatile uint8_t write_index;
    volatile pny_capture_sample_t samples[PNY_CAPTURE_MAX_SAMPLES];
} capture_state_t;

/* Type-3 tracking loop in mechanical turn32 units (2^-32 turn), per sensor
 * tick. Holding acceleration as a state removes the lag a type-2 loop has
 * while speed changes: that lag over-advanced commutation while braking and
 * made braking self-sustaining. */
typedef struct {
    uint32_t pos;
    int32_t vel;
    int32_t acc;
    uint8_t init;
} observer_state_t;

typedef struct {
    volatile bool active;
    volatile uint8_t sector;
    volatile int8_t step;
    volatile uint32_t period_q8;
    volatile uint16_t phase;
    volatile uint16_t phase_tick;
    volatile int32_t phase_rate_q8;
    volatile uint16_t next_comm_tick;
    volatile uint16_t position_tick;
    volatile uint16_t sample_limit_us;
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
#if PNY_DRIVE_BRUSHED
static volatile int32_t velocity_turn32_per_s;
#endif
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
#if PNY_DRIVE_BRUSHED
typedef int32_t control_gain_t;
#else
typedef int16_t control_gain_t;
#endif
static volatile control_gain_t control_kp_q8;
static volatile control_gain_t control_kd_q8;
static volatile int16_t control_kv_q8;
static volatile int16_t control_kf;
static volatile int16_t control_clip = DEFAULT_CONTROL_CLIP;
static volatile uint16_t control_deadband_turn32;
static volatile bool report_speed;
static volatile int32_t current_lead_turn16 = LEAD_DEFAULT_TURN16;
static pny_comm_settings_t comm_settings = {
    OBSERVER_LEAD_US,
    PNY_COMM_FLAG_GUARD | PNY_COMM_FLAG_I2C_FIRST,
    OBS_ALPHA_SHIFT, OBS_BETA_SHIFT, OBS_GAMMA_SHIFT,
    FW_DEFAULT_US, FW_MAX_DEFAULT_DEG,
};
#define observer_lead_us comm_settings.lead_us
static int32_t fw_max_turn16 = (FW_MAX_DEFAULT_DEG * 65536 + 180) / 360;
static uint16_t guard_ticks;
static pny_comm_diag_t comm_diag;
static uint32_t pending_reset_ms;

static volatile uint16_t isr_duration_us;
static volatile uint16_t isr_max_us;
static volatile uint32_t isr_overrun_count;
static volatile uint16_t sensor_i2c_us;
static volatile uint16_t sensor_i2c_start_us;
static volatile uint16_t sensor_i2c_end_us;
static uint16_t mct_fault_count;
static uint32_t next_mct_check_ms;
static uint32_t uart_overrun_errors;

static work_state_t work_state;
static observer_state_t observer_state;
static comm_scheduler_t comm_scheduler;
#if PNY_DRIVE_BRUSHED
static int32_t brushed_velocity_position;
static uint16_t brushed_velocity_tick;
static uint8_t brushed_velocity_sample_count;
#endif

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

static const mct_reg_value_t mct_expected_config[] = {
    {MCT8316Z_REG_CONTROL2A, MCT_EXPECTED_CONTROL2A},
    {MCT8316Z_REG_CONTROL3, MCT_EXPECTED_CONTROL3},
    {MCT8316Z_REG_CONTROL4, MCT_EXPECTED_CONTROL4},
    {MCT8316Z_REG_CONTROL5, MCT_EXPECTED_CONTROL5},
    {MCT8316Z_REG_CONTROL6, MCT_EXPECTED_CONTROL6},
    {MCT8316Z_REG_CONTROL8, MCT_EXPECTED_CONTROL8},
    {MCT8316Z_REG_CONTROL9, MCT_EXPECTED_CONTROL9},
    {MCT8316Z_REG_CONTROL10, MCT_EXPECTED_CONTROL10},
};

#if !PNY_DRIVE_BRUSHED
static const uint16_t comm_sector_start_turn16[] = {
    0x0000u,
    0x2aabu,
    0x5556u,
    0x8000u,
    0xaaabu,
    0xd556u,
    0x0000u,
};
#endif

#define cal_state work_state.cal
#define capture_state work_state.capture

/* Parsing finishes before a response is built. Reuse the receive buffer for
 * status to keep UART replies off the stack while three IRQs can nest. */
static union {
    pny_frame_parser_t parser;
    pny_status_payload_t status;
    pny_capture_read_payload_t capture_reply;
} uart_state;
#define frame_parser uart_state.parser
static uint32_t uart_last_frame_ms;
static volatile uint8_t uart_dma_rx[UART_DMA_RX_BUF_SIZE];
static uint16_t uart_dma_tail;
static bool sensor_run_mode;

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
    /* SysTick only keeps milliseconds. Sensor ticks come from TIM21 CC1 and
     * the estimator runs in PendSV, below I2C and commutation. */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(31999u);
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0xC0);
    nvic_set_priority(NVIC_PENDSV_IRQ, 0x80);
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
    /* Unsigned subtraction keeps the full distance across INT32_MIN/MAX. */
    if (target > current) {
        return (uint32_t)target - (uint32_t)current > (uint32_t)deadband ? 1 : 0;
    }
    return (uint32_t)current - (uint32_t)target > (uint32_t)deadband ? -1 : 0;
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

static uint16_t approx_hypot_u16(uint32_t x, uint32_t y)
{
    uint32_t maxv = (x > y) ? x : y;
    uint32_t minv = (x > y) ? y : x;
    uint32_t value = maxv + (minv >> 1);
    return (value > 0xFFFFu) ? 0xFFFFu : (uint16_t)value;
}

static uint32_t abs_u32(int32_t value)
{
    return (value < 0) ? -(uint32_t)value : (uint32_t)value;
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
    rcc_set_peripheral_clk_sel(I2C1, RCC_CCIPR_I2CxSEL_PCLK);
    I2C_TIMINGR(I2C1) = TMAG5273_I2C_TIMING;
    i2c_peripheral_enable(I2C1);
    tmag5273_async_setup();
    nvic_enable_irq(NVIC_I2C1_IRQ);
    /* Above TIM21 (PNY_COMM_FLAG_I2C_FIRST): with DMA receiving the data,
     * I2C needs two ~1 us interrupts per tick. A trigger-to-read restart
     * delayed behind a sector edge ends the read after the conversion it
     * started, and the TMAG discards that result. */
    nvic_set_priority(NVIC_I2C1_IRQ, 0x00);
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
    nvic_set_priority(NVIC_TIM21_IRQ, 0x40);
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

static __attribute__((noinline)) void set_hall_outputs_raw(uint8_t step)
{
    static const uint16_t pc_pins[6] = {
        HALLA_PIN | HALLB_PIN, HALLA_PIN, HALLA_PIN, 0, HALLB_PIN, HALLB_PIN
    };
    uint16_t pc_set = pc_pins[step];
    uint16_t pa_set = (step >= 2u && step <= 4u) ? HALLC_PIN : 0u;

    /* Adjacent sectors change one bit. Never pulse all Hall inputs low. */
    GPIO_BSRR(GPIOC) = pc_set | ((uint32_t)((HALLA_PIN | HALLB_PIN) & ~pc_set) << 16);
    GPIO_BSRR(GPIOA) = pa_set | ((uint32_t)(HALLC_PIN & ~pa_set) << 16);
}

static __attribute__((noinline)) void apply_pwm_duty(int16_t duty)
{
    uint16_t pwm = (duty < 0) ? (uint16_t)(-duty) : (uint16_t)duty;
    if (pwm > DUTY_LIMIT) {
        pwm = DUTY_LIMIT;
    }
    pwm = (uint16_t)(((uint32_t)pwm * PWM_PERIOD + (DUTY_LIMIT / 2u)) / DUTY_LIMIT);
    /* A sector fault may preempt a control calculation. Only the register
     * write is masked, so that calculation cannot re-enable PWM afterward. */
    nvic_disable_irq(NVIC_TIM21_IRQ);
    if (current_mode == PNY_MODE_IDLE) {
        pwm = 0;
        current_duty = 0;
    }
    timer_set_oc_value(TIM2, TIM_OC4, pwm);
    nvic_enable_irq(NVIC_TIM21_IRQ);
}

static void clear_motor_outputs(void)
{
    TIM_CCR4(TIM2) = 0u;
    gpio_clear(HALLA_PORT, HALLA_PIN);
    gpio_clear(HALLB_PORT, HALLB_PIN);
    gpio_clear(HALLC_PORT, HALLC_PIN);
    current_duty = 0;
    current_direction = 0;
}

static void release_brake(void)
{
    gpio_clear(BRAKE_PORT, BRAKE_PIN);
}

static void apply_brake(void)
{
    gpio_set(BRAKE_PORT, BRAKE_PIN);
}

static void stop_motor_outputs(void)
{
    clear_motor_outputs();
    observer_state.init = 0u;
}

static void velocity_reset(int32_t position)
{
    (void)position;
    observer_state.init = 0u;
    /* No field weakening or slip detection while the observer settles. */
    guard_ticks = GUARD_TICKS;
#if PNY_DRIVE_BRUSHED
    velocity_turn32_per_s = 0;
    brushed_velocity_position = position;
    brushed_velocity_tick = sched_now_us();
    brushed_velocity_sample_count = 0u;
#endif
}

#if PNY_DRIVE_BRUSHED
static void brushed_velocity_update(int32_t position, uint16_t tick)
{
    brushed_velocity_sample_count++;
    if (brushed_velocity_sample_count < BRUSHED_VELOCITY_UPDATE_SAMPLES) {
        return;
    }
    brushed_velocity_sample_count = 0u;

    uint16_t dt_us = (uint16_t)(tick - brushed_velocity_tick);
    if (dt_us > 30000u || dt_us < 10u) {
        brushed_velocity_position = position;
        brushed_velocity_tick = tick;
        return;
    }

    int32_t delta = position - brushed_velocity_position;
    int32_t velocity = (delta * 100000) / (int32_t)(dt_us / 10u);
    velocity_turn32_per_s += (velocity - velocity_turn32_per_s) / 4;
    brushed_velocity_position = position;
    brushed_velocity_tick = tick;
}
#endif

/* Position and sensor sampling */
#if !PNY_DRIVE_BRUSHED
/* Advance the observer to a sample stamped at `stamp` and correct it.
 * Returns the innovation (measured minus predicted) in turn16. */
static int16_t observer_update(uint16_t angle_turn16, uint16_t stamp)
{
    observer_state_t *o = &observer_state;
    uint16_t dt = (uint16_t)(stamp - comm_scheduler.position_tick);
    uint32_t measured = (uint32_t)angle_turn16 << 16;
    comm_scheduler.position_tick = stamp;

    int32_t k = 1;
    while (k < 8 && dt > (uint16_t)(k * SENSOR_TICK_US + SENSOR_TICK_US / 2u)) {
        k++;
    }
    if (o->init == 0u || dt >= SENSOR_STALE_US) {
        o->pos = measured;
        o->vel = 0;
        o->acc = 0;
        o->init = 1u;
        return 0;
    }
    if (o->init == 1u) {
        /* Second sample: start at the measured speed, so starting while the
         * rotor already spins does not begin with a large tracking error.
         * Division-free, so it waits for two consecutive samples. */
        if (k == 1) {
            o->vel = (int32_t)(measured - o->pos);
            o->init = 2u;
        }
        o->pos = measured;
        return 0;
    }
    int32_t vel = o->vel;
    int32_t acc = o->acc;
    uint32_t predicted = o->pos + (uint32_t)vel * (uint32_t)k +
                         (uint32_t)((acc >> 1) * k * k);
    int32_t error = (int32_t)(measured - predicted);
    if ((error > OBS_GATE_TURN32 || error < -OBS_GATE_TURN32) && o->init == 2u) {
        o->init = 3u;   /* coast on the prediction once */
        comm_diag.rejected++;
        o->pos = predicted;
        o->vel = vel + acc * k;
        return (int16_t)(error >> 16);
    }
    o->init = 2u;
    /* After missed samples the prediction spans k ticks: scale the rate
     * corrections by about 1/k and 1/k^2 (shifts; exact for k = 1, 2, 4). */
    uint8_t ks = (uint8_t)(k >= 4 ? 2 : k - 1);
    int32_t dv = error >> (comm_settings.beta_shift + ks);
    int32_t da = error >> (comm_settings.gamma_shift + 2u * ks);
    o->pos = predicted + (uint32_t)(error >> comm_settings.alpha_shift);
    o->vel = vel + acc * k + dv;
    o->acc = acc + da;
    return (int16_t)(error >> 16);
}

#endif

/* Unit conversions for readers (status, control), not the per-tick path. */
static int32_t velocity_now(void)
{
#if PNY_DRIVE_BRUSHED
    return velocity_turn32_per_s;
#else
    if (current_mode != PNY_MODE_RUN || observer_state.init < 2u) {
        return 0;
    }
    return (int32_t)(((int64_t)observer_state.vel * VEL_TURN16_PER_S_MUL) >> 16);
#endif
}

static int16_t accel_krpm_s(void)
{
    int32_t accel = (int32_t)(((int64_t)observer_state.acc * ACCEL_KRPM_MUL) >> 22);
    if (accel > INT16_MAX) accel = INT16_MAX;
    if (accel < -INT16_MAX) accel = -INT16_MAX;
    return (int16_t)accel;
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
    if (!target_position_set ||
        position_target_direction(
            target_position_turn32,
            position_from_zero(absolute_position_turn32),
            POSITION_DEADBAND_TURN32) != 0) {
        return false;
    }
#if PNY_DRIVE_BRUSHED
    return velocity_now() > -VELOCITY_DEADBAND_TURN32_PER_S &&
           velocity_now() < VELOCITY_DEADBAND_TURN32_PER_S;
#else
    return true;
#endif
}

static bool encoder_angle_valid(void)
{
#if PNY_DRIVE_BRUSHED
    return true;
#else
    return pennyesc_calibration_valid();
#endif
}

static uint16_t sensor_angle_turn16(int16_t x, int16_t y)
{
#if PNY_DRIVE_BRUSHED
    return pennyesc_raw_angle_turn16(x, y);
#else
    if (pennyesc_calibration_valid()) {
        return pennyesc_calibration_angle_turn16(x, y);
    }
    return 0u;
#endif
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
#if PNY_DRIVE_BRUSHED
    if (!tmag5273_set_mode(TMAG5273_MODE_FAST_XYZ)) {
#else
    if (!tmag5273_set_mode(TMAG5273_MODE_TRIGGER_XY)) {
#endif
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

    if (encoder_angle_valid()) {
        uint16_t angle = sensor_angle_turn16(x, y);
        current_angle_turn16 = angle;
        if (update_position) {
            update_position_from_angle(angle);
        }
    } else {
        current_angle_turn16 = 0;
    }
}

#if PNY_DRIVE_BRUSHED
static bool refresh_sensor_xy(bool update_position)
{
    int16_t x;
    int16_t y;
    int16_t z;

    if (!tmag5273_read_fast(&x, &y, &z)) {
        sensor_ready = false;
        current_angle_turn16 = 0;
        return false;
    }
    sensor_ready = true;
    sensor_x = x;
    sensor_y = y;
    sensor_z = z;

    uint16_t angle = sensor_angle_turn16(x, y);
    current_angle_turn16 = angle;
    if (update_position) {
        update_position_from_angle(angle);
    }
    return true;
}

#endif

static void apply_sensor_xy_sample(const tmag5273_xy_sample_t *sample, bool update_position)
{
    sensor_ready = true;
    sensor_x = sample->x;
    sensor_y = sample->y;
    sensor_z = sample->z;
    sensor_i2c_start_us = sample->start_phase_us;
    sensor_i2c_end_us = sample->end_phase_us;
    sensor_i2c_us = (uint16_t)(sample->end_phase_us - sample->start_phase_us);
    uint16_t angle = sensor_angle_turn16(sample->x, sample->y);
    current_angle_turn16 = angle;
    if (update_position) {
        update_position_from_angle(angle);
    }
}

static int32_t lead_deg_to_turn16(int16_t lead_deg)
{
    /* 65536 / 360 = 46603.4 / 256, without a signed divide. */
    return ((int32_t)lead_deg * 46603 + 128) >> 8;
}

/* Commutation scheduler */
static void comm_fault(void);
#if !PNY_DRIVE_BRUSHED
/* Electrical commutation phase for a mechanical angle in turn32. fw is
 * extra advance in the drive direction. */
static uint16_t commutation_phase(uint32_t mech_turn32, int direction, int32_t fw)
{
    int32_t phase = (int32_t)((mech_turn32 * POLE_PAIRS) >> 16) +
                    pennyesc_calibration_commutation_alignment_turn16() +
                    current_lead_turn16;
    if (direction < 0) {
        phase -= REVERSE_HALL_PHASE_TURN16 + fw;
    } else {
        phase += fw;
    }
    return (uint16_t)phase;
}

static uint16_t comm_sector_start_phase(uint8_t sector)
{
    return comm_sector_start_turn16[sector];
}

/* Called with TIM21 masked or from its ISR. Clear the old event before
 * writing CCR, then recheck the clock: an expired compare needs a software
 * event, not a wait for the 65 ms counter wrap. */
static __attribute__((noinline)) void comm_arm(uint16_t deadline)
{
    TIM_DIER(TIM21) &= ~TIM_DIER_CC2IE;
    timer_clear_flag(TIM21, TIM_SR_CC2IF);
    comm_scheduler.next_comm_tick = deadline;
    TIM_CCR2(TIM21) = deadline;
    TIM_DIER(TIM21) |= TIM_DIER_CC2IE;
    if (tick_due(sched_now_us(), deadline)) {
        timer_generate_event(TIM21, TIM_EGR_CC2G);
    }
}

static void comm_schedule_absolute(uint16_t now)
{
    uint16_t elapsed = (uint16_t)(now - comm_scheduler.phase_tick);
    uint16_t phase = (uint16_t)(comm_scheduler.phase +
        ((comm_scheduler.phase_rate_q8 * (int32_t)elapsed) >> 8));
    uint8_t sector = (uint8_t)(((uint32_t)phase * 6u) >> 16);
    if (sector != comm_scheduler.sector) {
        set_hall_outputs_raw(sector);
        comm_scheduler.sector = sector;
    }
    int8_t step = comm_scheduler.step;
    uint16_t boundary = step > 0 ? comm_sector_start_phase(sector_next(sector, 1)) :
        (uint16_t)(comm_sector_start_phase(sector) - 1u);
    uint16_t distance = step > 0 ? (uint16_t)(boundary - phase) : (uint16_t)(phase - boundary);
    /* The angle is authoritative at every event. Integer period rounding
     * changes edge jitter by <1 us; it cannot accumulate into phase drift. */
    uint32_t period = comm_scheduler.period_q8 >> 8;
    uint32_t dt = ((uint32_t)distance * 6u * period + 65535u) >> 16;
    if (dt == 0u) dt = 1u;
    comm_arm((uint16_t)(now + dt));
}

static void comm_scheduler_schedule_sector_event(uint16_t now, int direction)
{
    /* Extrapolate the observer to now + lead with its acceleration. */
    /* 32-bit products of pre-shifted terms: the horizon resolves to 0.5 us
     * and the rate to 5e-5, both well inside the sensor's resolution. */
    int32_t h = (int16_t)(now - comm_scheduler.position_tick) + observer_lead_us;
    int32_t hq = (h * TICK_Q24_PER_US) >> 16;   /* Q8 sensor ticks */
    int32_t vel = observer_state.vel;
    int32_t velocity = vel + ((observer_state.acc >> 8) * hq);
    uint32_t mech = observer_state.pos + (uint32_t)(((vel >> 1) + (velocity >> 1)) >> 8) * (uint32_t)hq;
    int32_t rate = ((velocity >> 14) * (int32_t)(RATE_Q8_MUL >> 6)) >> 12;
    uint32_t rate_abs = abs_u32(rate);
    uint32_t period = rate_abs != 0u ? SECTOR_TURN16_Q16 / rate_abs : 0u;
    if (period > (COMM_MAX_PERIOD_US << 8)) {
        period = 0u;
    }
    /* Field-weakening advance grows with speed but stays below the torque
     * peak; past it, more advance lowers torque and the loop can run away. */
    int32_t fw = 0;
    if (guard_ticks == 0u && (velocity > 0) == (direction > 0)) {
        fw = ((int32_t)rate_abs * comm_settings.fw_us) >> 8;
        if (fw > fw_max_turn16) fw = fw_max_turn16;
        if (fw < -fw_max_turn16) fw = -fw_max_turn16;
    }
    comm_diag.fw_deg = (int16_t)((fw * 45) >> 13);
    uint16_t phase = commutation_phase(mech, direction, fw);

    nvic_disable_irq(NVIC_TIM21_IRQ);
    if (current_mode != PNY_MODE_RUN || direction == 0 || current_duty == 0) {
        TIM_DIER(TIM21) &= ~TIM_DIER_CC2IE;
    } else if (period == 0u) {
        comm_scheduler.sample_limit_us = SENSOR_STALE_US;
        TIM_DIER(TIM21) &= ~TIM_DIER_CC2IE;
        uint8_t sector = (uint8_t)(((uint32_t)phase * 6u) >> 16);
        if (sector != comm_scheduler.sector) {
            set_hall_outputs_raw(sector);
            comm_scheduler.sector = sector;
        }
    } else if (period < (COMM_MIN_PERIOD_US << 8)) {
        comm_fault();
    } else {
        comm_scheduler.period_q8 = period;
        comm_scheduler.phase = phase;
        comm_scheduler.phase_tick = now;
        comm_scheduler.phase_rate_q8 = rate;
        comm_scheduler.step = velocity >= 0 ? 1 : -1;
        uint32_t limit = (period >> 8) * (3u * POLE_PAIRS);
        comm_scheduler.sample_limit_us = limit < SENSOR_STALE_US ? (uint16_t)limit : SENSOR_STALE_US;
        /* Include time spent calculating above; publishing an old phase as
         * current would introduce a speed-dependent commutation error. */
        comm_schedule_absolute(sched_now_us());
    }
    nvic_enable_irq(NVIC_TIM21_IRQ);
}
#endif

static void comm_scheduler_stop(void)
{
    TIM_DIER(TIM21) &= ~(TIM_DIER_CC1IE | TIM_DIER_CC2IE);
    comm_scheduler.active = false;
    comm_scheduler.sector = 0xffu;
    comm_scheduler.last_duty = INT16_MIN;
    tmag5273_async_cancel();
}

static void comm_scheduler_start(void)
{
    uint16_t now = sched_now_us();
    comm_scheduler.sector = 0xffu;
    comm_scheduler.position_tick = now;
    comm_scheduler.sample_limit_us = SENSOR_STALE_US;
    comm_scheduler.last_duty = INT16_MIN;
    comm_scheduler.next_comm_tick = now;
    TIM_DIER(TIM21) &= ~TIM_DIER_CC2IE;
    timer_clear_flag(TIM21, TIM_SR_CC1IF | TIM_SR_CC2IF);
    /* TIM21 CC1 starts every sensor transaction at a fixed period. */
    TIM_CCR1(TIM21) = (uint16_t)(now + 10u);
    comm_scheduler.active = true;
    TIM_DIER(TIM21) |= TIM_DIER_CC1IE;
}

static bool sensor_start_read(void)
{
    for (uint8_t i = 0; i < SENSOR_START_RETRIES; i++) {
        if (!sensor_ready && !sensor_init_full_mode()) {
            delay_ms(1);
            continue;
        }
#if PNY_DRIVE_BRUSHED
        if (sensor_set_run_mode() && refresh_sensor_xy(false)) {
            return true;
        }
#else
        /* Take the initial angle before enabling fast continuous XY reads. */
        if (sensor_set_full_mode()) {
            refresh_sensor_xyz(false);
            if (sensor_ready && sensor_set_run_mode()) {
                return true;
            }
        }
#endif
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
    if (!encoder_angle_valid()) {
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

static bool mct_config_valid(void)
{
    if ((mct8316z_read_reg(MCT8316Z_REG_IC_STATUS) & MCT_IC_STATUS_NPOR) == 0u) {
        return false;
    }
    for (uint8_t i = 0; i < sizeof(mct_expected_config) / sizeof(mct_expected_config[0]); i++) {
        if ((uint8_t)mct8316z_read_reg(mct_expected_config[i].reg) != mct_expected_config[i].value) {
            return false;
        }
    }
    uint8_t control7 = (uint8_t)mct8316z_read_reg(MCT8316Z_REG_CONTROL7);
    return (control7 & (MCT8316Z_HALL_HYS_HIGH | MCT8316Z_DIR_REVERSE)) == MCT8316Z_HALL_HYS_HIGH;
}

static void mct_recover_if_needed(void)
{
    if (mct_config_valid()) {
        return;
    }

    bool restart_timer = current_mode == PNY_MODE_RUN;
    if (restart_timer) {
        comm_scheduler_stop();
    }
    apply_pwm_duty(0);
    delay_ms(2);

    if (mct_config_valid()) {
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

static __attribute__((noinline)) int32_t control_term(control_gain_t gain_q8, int32_t error_turn32)
{
    return (int32_t)(((int64_t)(gain_q8 * CONTROL_TURN32_NUM) * error_turn32) >> CONTROL_TURN32_SHIFT);
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

#if PNY_DRIVE_BRUSHED
static int16_t brushed_friction_duty(int32_t position_error)
{
    if (!target_position_set || position_error == 0) {
        return 0;
    }

    if (position_error >= BRUSHED_FRICTION_RAMP_TURN32) {
        return BRUSHED_FRICTION_DUTY;
    }
    if (position_error <= -BRUSHED_FRICTION_RAMP_TURN32) {
        return -BRUSHED_FRICTION_DUTY;
    }

    int32_t magnitude = position_error < 0 ? -position_error : position_error;
    magnitude = (magnitude * BRUSHED_FRICTION_DUTY) / BRUSHED_FRICTION_RAMP_TURN32;
    return position_error > 0 ? (int16_t)magnitude : (int16_t)-magnitude;
}
#endif

static int16_t control_duty(void)
{
#if !PNY_DRIVE_BRUSHED
    if ((control_kp_q8 | control_kd_q8 | control_kv_q8) == 0) {
        return clamp_duty_i32(control_kf);
    }
#endif
    int32_t position_error = target_position_turn32 - position_from_zero(absolute_position_turn32);
    int32_t velocity_error = target_velocity_turn32_per_s - velocity_now();
#if PNY_DRIVE_BRUSHED
    if (target_position_set && target_velocity_turn32_per_s == 0 &&
        position_error >= -BRUSHED_POSITION_HOLD_TURN32 &&
        position_error <= BRUSHED_POSITION_HOLD_TURN32) {
        return 0;
    }
#else
    if (position_error > control_deadband_turn32) {
        position_error -= control_deadband_turn32;
    } else if (position_error < -(int32_t)control_deadband_turn32) {
        position_error += control_deadband_turn32;
    } else {
        position_error = 0;
    }
#endif
    int32_t duty =
        control_term(control_kp_q8, position_error) +
        control_term(control_kd_q8, velocity_error) +
        control_term(control_kv_q8, target_velocity_turn32_per_s) +
        control_kf;
#if PNY_DRIVE_BRUSHED
    duty += brushed_friction_duty(position_error);
#endif
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

    int16_t duty;
    int8_t direction;
    get_drive_command(&duty, &direction);

#if PNY_DRIVE_BRUSHED
    mct_recover_if_needed();
    next_mct_check_ms = system_millis + MCT_IDLE_CHECK_MS;
    clear_motor_outputs();
    current_direction = direction != 0 ? direction : 1;
    set_hall_outputs_raw(current_direction > 0 ? BRUSHED_FORWARD_STEP : BRUSHED_REVERSE_STEP);
    release_brake();
    current_mode = PNY_MODE_RUN;
    comm_scheduler_start();
    return;
#else
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
    uint8_t sector = (uint8_t)(((uint32_t)commutation_phase(
        (uint32_t)current_angle_turn16 << 16, direction, 0) * 6u) >> 16);
    set_hall_outputs_raw(sector);
    current_mode = PNY_MODE_RUN;
    apply_pwm_duty(duty);
    comm_scheduler_start();
    comm_scheduler.sector = sector;
    /* The observer initializes from the first two samples; PendSV arms edges. */
#endif
}

static uint8_t control_apply(void)
{
    if (current_mode == PNY_MODE_CAL) {
        return PNY_RESULT_BUSY;
    }

    release_brake();

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
    if (!encoder_angle_valid()) {
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
    if (encoder_angle_valid()) {
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
    payload->result = result;
    payload->mode = current_mode;
    payload->faults = current_faults();
    payload->flags = current_flags(payload->faults);
    payload->x = sensor_x;
    payload->y = sensor_y;
    /* While running, z carries the observer's acceleration in krpm/s. */
    payload->z = current_mode == PNY_MODE_RUN ? accel_krpm_s() : sensor_z;
    payload->angle_turn16 = current_angle_turn16;
    payload->position_turn32 = position_from_zero(absolute_position_turn32);
    payload->velocity_turn32_per_s = velocity_now();
    payload->duty = current_duty;
    payload->mct_fault_count = mct_fault_count;
    payload->isr_us = (uint16_t)isr_duration_us;
    payload->isr_max_us = (uint16_t)isr_max_us;
    payload->i2c_us = (uint16_t)sensor_i2c_us;
    payload->i2c_start_us = sensor_i2c_start_us;
    payload->i2c_end_us = sensor_i2c_end_us;
    payload->isr_overrun_count = isr_overrun_count;
    tmag5273_stats_t tmag_stats;
    tmag5273_get_stats(&tmag_stats);
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

static void send_status_response(uint8_t cmd, uint8_t result)
{
    fill_status_payload(&uart_state.status, result);
    send_frame(cmd, &uart_state.status, sizeof(uart_state.status));
}

static void handle_get_status(void)
{
    refresh_status_sensor();
    send_status_response(PNY_CMD_GET_STATUS, PNY_RESULT_OK);
}

/* Stop the motor, acknowledge, and reset into the UART bootloader. */
static void handle_enter_boot(const uint8_t *payload, uint8_t len)
{
    uint32_t magic = 0u;
    uint8_t result = PNY_RESULT_OK;

    if (len == 4u) {
        memcpy(&magic, payload, sizeof(magic));
    }
    if (magic != PNY_BOOT_MAGIC) {
        result = PNY_RESULT_BAD_ARG;
    } else if (current_mode == PNY_MODE_CAL) {
        result = PNY_RESULT_BUSY;
    } else {
        control_disable();
        run_stop();
        pending_reset_ms = system_millis + 20u;
    }
    send_frame(PNY_CMD_ENTER_BOOT, &result, 1u);
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
#if PNY_DRIVE_BRUSHED
    (void)sweep_dir;
    return PNY_RESULT_BAD_STATE;
#else
    if (current_mode == PNY_MODE_CAL) {
        return PNY_RESULT_BAD_STATE;
    }
    if (sweep_dir > 1u) {
        return PNY_RESULT_BAD_ARG;
    }

    memset(&cal_state, 0, sizeof(cal_state));
    control_disable();
    run_stop();
    if (!sensor_init_full_mode()) {
        return PNY_RESULT_BAD_STATE;
    }
    mct_apply_config();
    delay_ms(2);
    current_mode = PNY_MODE_CAL;
    release_brake();
    cal_state.active = true;
    cal_state.total_points = PNY_CAL_POINTS_PER_SWEEP;
    cal_state.sweep_dir = sweep_dir;
    calibration_arm_point(0);
    return PNY_RESULT_OK;
#endif
}

static __attribute__((noinline)) void calibration_poll(void)
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
static void capture_finish(void)
{
    capture_state.active = false;
    capture_state.done = true;
    if (capture_state.owns_motor) {
        control_disable();
    }
}

/* One record per sensor tick: measured angle and innovation (INT16_MIN when
 * no fresh sample arrived). */
static void capture_record(uint16_t angle, int16_t innovation)
{
    if (!capture_state.active) {
        return;
    }
    if (++capture_state.decim_count < capture_state.decimation) {
        return;
    }
    capture_state.decim_count = 0u;
    uint8_t slot = capture_state.write_index;
    capture_state.samples[slot].angle_turn16 = angle;
    capture_state.samples[slot].rpm = innovation;
    capture_state.write_index = (uint8_t)(slot + 1u == PNY_CAPTURE_MAX_SAMPLES ? 0u : slot + 1u);
    if (capture_state.sample_count < PNY_CAPTURE_MAX_SAMPLES) {
        capture_state.sample_count++;
    }
    if (capture_state.trigger_index < PNY_CAPTURE_MAX_SAMPLES) {
        capture_state.trigger_index++;   /* records since the trigger */
    }
    if ((capture_state.post_trigger != 0u && --capture_state.post_trigger == 0u) ||
        (capture_state.duration_ms != 0u &&
         (uint32_t)(system_millis - capture_state.start_ms) >= capture_state.duration_ms)) {
        capture_finish();
    }
}

static void capture_trigger(void)
{
    if (capture_state.active && capture_state.trigger_index == UINT16_MAX) {
        capture_state.trigger_index = 0u;
        capture_state.post_trigger = PNY_CAPTURE_MAX_SAMPLES / 4u;
    }
}

static void comm_stop_from_isr(void)
{
    current_mode = PNY_MODE_IDLE;
    TIM_DIER(TIM21) &= ~TIM_DIER_CC2IE;
    stop_motor_outputs();
    /* Main finishes sensor cleanup; TIM21 may have preempted its I2C ISR. */
}

static void comm_fault(void)
{
    sensor_ready = false;
    comm_diag.comm_faults++;
    if (capture_state.active) {
        capture_trigger();
        capture_finish();
    }
    control_disable();
    comm_stop_from_isr();
}

#if PNY_DRIVE_BRUSHED
static void brushed_set_direction(int8_t direction)
{
    set_hall_outputs_raw(direction > 0 ? BRUSHED_FORWARD_STEP : BRUSHED_REVERSE_STEP);
}

static void brushed_control_tick(int16_t duty, int8_t direction)
{
    if (duty == 0 || direction == 0) {
        current_duty = 0;
        apply_pwm_duty(0);
        return;
    }

    if (current_direction != direction) {
        brushed_set_direction(direction);
        current_direction = direction;
    }

    current_duty = duty;
    apply_pwm_duty(duty);
}
#endif

static void comm_control_tick(uint16_t now)
{
    int16_t duty;
    int8_t direction;
    get_drive_command(&duty, &direction);

#if PNY_DRIVE_BRUSHED
    (void)now;
    brushed_control_tick(duty, direction);
    if (duty == 0 && !capture_state.active && !report_speed &&
        control_kp_q8 == 0 && control_kd_q8 == 0 && control_kv_q8 == 0 && control_kf == 0) {
        current_mode = PNY_MODE_IDLE;
        comm_scheduler_stop();
    }
    return;
#else
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

    if (direction != current_direction) {
        current_direction = direction;
        comm_scheduler_schedule_sector_event(now, direction);
    }

    if (duty != comm_scheduler.last_duty) {
        apply_pwm_duty(duty);
        comm_scheduler.last_duty = duty;
    }
#endif
}

#if !PNY_DRIVE_BRUSHED
static int16_t comm_observe(const tmag5273_xy_sample_t *sample)
{
    int16_t e = observer_update(current_angle_turn16,
                                (uint16_t)(sample->sample_tick + TRIGGER_SAMPLE_OFFSET_US));
    uint16_t mag = (uint16_t)(e < 0 ? -e : e);
    comm_diag.innov_mean_x16 = (uint16_t)(comm_diag.innov_mean_x16 +
        (((int32_t)mag * 16 - comm_diag.innov_mean_x16) >> 5));
    if (mag > comm_diag.innov_max) {
        comm_diag.innov_max = mag;
    }

    /* Hard deceleration against the drive means commutation slipped past
     * the torque peak. Drop field-weakening advance until it recovers. */
    int32_t a = current_direction < 0 ? -observer_state.acc : observer_state.acc;
    if ((comm_settings.flags & PNY_COMM_FLAG_GUARD) != 0u && current_duty != 0 &&
        a < -GUARD_ACCEL_TURN32 && abs_u32(observer_state.vel) > GUARD_MIN_TURN32) {
        if (guard_ticks == 0u) {
            comm_diag.guard_events++;
            capture_trigger();
        }
        guard_ticks = GUARD_TICKS;
    } else if (guard_ticks != 0u) {
        guard_ticks--;
    }
    return e;
}
#endif

static void comm_sensor_tick(void)
{
    tmag5273_xy_sample_t sample;
    bool have_sample = tmag5273_async_take_xy(&sample);
    int16_t innovation = INT16_MIN;

    if (have_sample) {
        comm_diag.samples++;
        apply_sensor_xy_sample(&sample, true);
#if PNY_DRIVE_BRUSHED
        brushed_velocity_update(absolute_position_turn32, sample.sample_tick);
        comm_scheduler.position_tick = sample.sample_tick;
#else
        innovation = comm_observe(&sample);
#endif
    } else {
        comm_diag.missed_ticks++;
    }

    if (current_mode != PNY_MODE_RUN) {
        return;
    }
    uint16_t control_tick = sched_now_us();
    if ((uint16_t)(control_tick - comm_scheduler.position_tick) > comm_scheduler.sample_limit_us) {
        comm_fault();
        return;
    }

    capture_record(current_angle_turn16, innovation);
    comm_control_tick(control_tick);
#if !PNY_DRIVE_BRUSHED
    if (have_sample || (TIM_DIER(TIM21) & TIM_DIER_CC2IE) == 0u) {
        comm_scheduler_schedule_sector_event(control_tick, current_direction);
    }
#endif
}

static void pend_estimator(void)
{
    SCB_ICSR = SCB_ICSR_PENDSVSET;
}

/* Sensor tick: start the TMAG transaction at a fixed period from the
 * highest-priority interrupt, so the trigger instant has ~1 us jitter. */
static void sensor_tick_isr(void)
{
    uint16_t now = sched_now_us();
    uint16_t next = (uint16_t)(TIM_CCR1(TIM21) + SENSOR_TICK_US);
    if (tick_due(now, next)) {
        next = (uint16_t)(now + SENSOR_TICK_US);
    }
    TIM_CCR1(TIM21) = next;
    if (!tmag5273_async_start_xy(now)) {
        isr_overrun_count++;
        pend_estimator();
    }
}

void tim21_isr(void)
{
    uint32_t flags = TIM_SR(TIM21) & TIM_DIER(TIM21);
    if ((flags & TIM_SR_CC1IF) != 0u) {
        timer_clear_flag(TIM21, TIM_SR_CC1IF);
        sensor_tick_isr();
    }
#if !PNY_DRIVE_BRUSHED
    if ((flags & TIM_SR_CC2IF) == 0u) {
        return;
    }
    uint16_t now = sched_now_us();
    if (current_duty == 0 || current_mode != PNY_MODE_RUN) {
        TIM_DIER(TIM21) &= ~TIM_DIER_CC2IE;
        timer_clear_flag(TIM21, TIM_SR_CC2IF);
        return;
    }
    if ((uint16_t)(now - comm_scheduler.position_tick) > comm_scheduler.sample_limit_us) {
        isr_overrun_count++;
        comm_fault();
        return;
    }
    /* Re-select from absolute angle even after a delayed interrupt. A free-
     * running sector counter can remain one or more sectors out of phase. */
    comm_schedule_absolute(now);
#endif
}

/* STM32L011 has TIM2 and TIM21, but no TIM22. TIM21 CC1 paces sensor reads,
 * CC2 places sector edges; SysTick only counts milliseconds. */
void sys_tick_handler(void)
{
    system_millis++;
}

/* Estimator and control, once per sensor transaction, below I2C and TIM21. */
void pend_sv_handler(void)
{
    if (current_mode != PNY_MODE_RUN || !comm_scheduler.active) {
        return;
    }
    uint16_t start = sched_now_us();
    comm_sensor_tick();
    isr_duration_us = (uint16_t)(sched_now_us() - start);
    if (isr_duration_us > isr_max_us) {
        isr_max_us = isr_duration_us;
    }
    if (isr_duration_us > comm_diag.proc_max_us) {
        comm_diag.proc_max_us = isr_duration_us;
    }
    if (isr_duration_us >= SENSOR_TICK_US) {
        isr_overrun_count++;
    }
}

void i2c1_isr(void)
{
    if (tmag5273_i2c1_isr()) {
        pend_estimator();
    }
}

/* UART protocol */

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

static void handle_brake(const uint8_t *payload, uint8_t len)
{
    (void)payload;

    if (len != 0u) {
        send_status_response(PNY_CMD_BRAKE, PNY_RESULT_BAD_ARG);
        return;
    }
    if (current_mode == PNY_MODE_CAL) {
        send_status_response(PNY_CMD_BRAKE, PNY_RESULT_BUSY);
        return;
    }

    control_disable();
    capture_state.active = false;
    clear_motor_outputs();
    if (current_mode != PNY_MODE_RUN && encoder_angle_valid() && run_ready() == PNY_RESULT_OK) {
        run_begin();
    }
    report_speed = current_mode == PNY_MODE_RUN;
    apply_brake();
    send_status_response(PNY_CMD_BRAKE, PNY_RESULT_OK);
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
    if (len != 4u && len != 10u) {
        return;
    }

    int32_t position_turn32;
    memcpy(&position_turn32, payload, sizeof(position_turn32));

    irq_lock();
    target_position_turn32 = position_turn32;
    if (len == 10u) {
        memcpy((void *)&target_velocity_turn32_per_s, payload + 4u, 4u);
        memcpy((void *)&control_kf, payload + 8u, 2u);
    }
    target_position_set = true;
    irq_unlock();
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
#if PNY_DRIVE_BRUSHED
    control_gain_t kp_q8;
    control_gain_t kd_q8;
    int16_t kv_q8;
    int16_t kf;
    int16_t clip;

    if (len == sizeof(pny_brushed_control_payload_t)) {
        pny_brushed_control_payload_t control;
        memcpy(&control, payload, sizeof(control));
        kp_q8 = control.kp_q8;
        kd_q8 = control.kd_q8;
        kv_q8 = control.kv_q8;
        kf = control.kf;
        clip = control.clip;
    } else if (len == sizeof(pny_control_payload_t)) {
        pny_control_payload_t control;
        memcpy(&control, payload, sizeof(control));
        kp_q8 = control.kp_q8;
        kd_q8 = control.kd_q8;
        kv_q8 = control.kv_q8;
        kf = control.kf;
        clip = control.clip;
    } else {
        send_status_response(PNY_CMD_SET_CONTROL, PNY_RESULT_BAD_ARG);
        return;
    }

    if (kp_q8 < -BRUSHED_CONTROL_GAIN_LIMIT_Q8 || kp_q8 > BRUSHED_CONTROL_GAIN_LIMIT_Q8 ||
        kd_q8 < -BRUSHED_CONTROL_GAIN_LIMIT_Q8 || kd_q8 > BRUSHED_CONTROL_GAIN_LIMIT_Q8 ||
        clip < 0 || clip > DUTY_LIMIT || kf < -DUTY_LIMIT || kf > DUTY_LIMIT) {
        send_status_response(PNY_CMD_SET_CONTROL, PNY_RESULT_RANGE);
        return;
    }

    irq_lock();
    control_kp_q8 = kp_q8;
    control_kd_q8 = kd_q8;
    control_kv_q8 = kv_q8;
    control_kf = kf;
    control_clip = clip;
    irq_unlock();
#else
    if (len != sizeof(pny_control_payload_t) && len != sizeof(pny_control_payload_t) + 2u) {
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

    irq_lock();
    control_kp_q8 = control.kp_q8;
    control_kd_q8 = control.kd_q8;
    control_kv_q8 = control.kv_q8;
    control_kf = control.kf;
    control_clip = control.clip;
    control_deadband_turn32 = len == sizeof(control) + 2u ?
        (uint16_t)(payload[sizeof(control)] | ((uint16_t)payload[sizeof(control) + 1u] << 8)) : 0u;
    irq_unlock();
#endif
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
    if (ok) {
        /* The new calibration is live; re-derive position from it. */
        position_initialized = false;
        current_angle_turn16 = 0;
        refresh_status_sensor();
    }
    out.result = ok ? PNY_RESULT_OK : PNY_RESULT_FLASH;
    out.valid = ok ? 1u : 0u;
    send_frame(PNY_CMD_CAL, &out, sizeof(out));
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

/* sample_hz reports the decimation (records every Nth sensor tick).
 * missed_count reports the first record after the trigger, in read order,
 * or 0xffff when nothing triggered. */
static void fill_capture_status(pny_capture_status_payload_t *out, uint8_t result)
{
    uint16_t stored = capture_state.sample_count;
    out->subcmd = PNY_DEBUG_CAPTURE_STATUS;
    out->result = result;
    out->active = capture_state.active ? 1u : 0u;
    out->done = capture_state.done ? 1u : 0u;
    out->sample_hz = capture_state.decimation;
    out->duration_ms = capture_state.duration_ms;
    out->elapsed_ms = (uint16_t)(system_millis - capture_state.start_ms);
    out->sample_count = stored;
    out->missed_count = capture_state.trigger_index > stored ? UINT16_MAX :
        (uint16_t)(stored - capture_state.trigger_index);
    out->mct_fault_count = mct_fault_count;
}

static void send_capture_status(uint8_t result)
{
    pny_capture_status_payload_t out;
    fill_capture_status(&out, result);
    send_frame(PNY_CMD_DEBUG, &out, sizeof(out));
}

/* duty 0 arms a recorder for whatever is running. A nonzero duty also runs
 * the motor at that duty and advance, and stops it when the capture ends.
 * sample_hz is the decimation (1 = every sensor tick). duration_ms 0 records
 * until a slip-guard event or fault. */
static __attribute__((noinline)) uint8_t capture_start(const uint8_t *payload, uint8_t len)
{
    if (len != sizeof(pny_capture_start_payload_t)) {
        return PNY_RESULT_BAD_ARG;
    }

    pny_capture_start_payload_t in;
    memcpy(&in, payload, sizeof(in));
    if (in.duration_ms > CAPTURE_MAX_DURATION_MS || in.sample_hz > 255u ||
        (in.duration_ms == 0u && in.duty != 0) ||
        in.advance_deg < ADVANCE_MIN_DEG || in.advance_deg > ADVANCE_MAX_DEG ||
        in.duty > DUTY_LIMIT || in.duty < -DUTY_LIMIT) {
        return PNY_RESULT_RANGE;
    }

    if (current_mode == PNY_MODE_CAL) {
        return PNY_RESULT_BUSY;
    }
    uint8_t result = run_ready();
    if (result != PNY_RESULT_OK) {
        return result;
    }

    irq_lock();
    memset(&capture_state, 0, sizeof(capture_state));
    capture_state.decimation = in.sample_hz != 0u ? (uint8_t)in.sample_hz : 1u;
    capture_state.duration_ms = in.duration_ms;
    capture_state.trigger_index = UINT16_MAX;
    capture_state.start_ms = system_millis;
    capture_state.owns_motor = in.duty != 0;
    capture_state.active = true;
    if (in.duty != 0) {
        current_lead_turn16 = lead_deg_to_turn16(in.advance_deg);
        control_set_duty(in.duty, DUTY_LIMIT);
        release_brake();
    }
    irq_unlock();
    if (in.duty != 0 && current_mode != PNY_MODE_RUN) {
        run_begin();
        if (current_mode != PNY_MODE_RUN) {
            capture_state.active = false;
            capture_state.done = true;
            control_disable();
            return PNY_RESULT_BAD_STATE;
        }
    }

    return PNY_RESULT_OK;
}

static __attribute__((noinline)) void handle_capture_read(const uint8_t *payload, uint8_t len)
{
    bool valid = len == 4u && payload[0] == PNY_DEBUG_CAPTURE_READ && payload[3] <= PNY_CAPTURE_READ_MAX_SAMPLES;
    uint16_t offset = valid ? (uint16_t)(payload[1] | ((uint16_t)payload[2] << 8)) : 0u;
    uint8_t want = valid ? payload[3] : 0u;
    /* Save the request before reusing its parser buffer for the reply. */
    pny_capture_read_payload_t *out = &uart_state.capture_reply;
    memset(out, 0, sizeof(*out));
    out->subcmd = PNY_DEBUG_CAPTURE_READ;

    if (!valid) {
        out->result = PNY_RESULT_BAD_ARG;
        send_frame(PNY_CMD_DEBUG, out, 5u);
        return;
    }
    if (capture_state.active) {
        out->result = PNY_RESULT_BUSY;
        send_frame(PNY_CMD_DEBUG, out, 5u);
        return;
    }

    uint16_t stored = capture_state.sample_count;
    if (offset > stored) {
        out->result = PNY_RESULT_RANGE;
        send_frame(PNY_CMD_DEBUG, out, 5u);
        return;
    }

    uint16_t available = (uint16_t)(stored - offset);
    if (want > available) {
        want = (uint8_t)available;
    }

    /* Oldest first: the oldest record sits at write_index once full. */
    uint16_t index = (uint16_t)((stored < PNY_CAPTURE_MAX_SAMPLES ? 0u : capture_state.write_index) + offset);
    if (index >= PNY_CAPTURE_MAX_SAMPLES) {
        index = (uint16_t)(index - PNY_CAPTURE_MAX_SAMPLES);
    }
    out->result = PNY_RESULT_OK;
    out->offset = offset;
    out->count = want;
    for (uint8_t i = 0; i < want; i++) {
        out->samples[i].angle_turn16 = capture_state.samples[index].angle_turn16;
        out->samples[i].rpm = capture_state.samples[index].rpm;
        if (++index == PNY_CAPTURE_MAX_SAMPLES) {
            index = 0u;
        }
    }
    send_frame(PNY_CMD_DEBUG, out, (uint8_t)(5u + (want * sizeof(pny_capture_sample_t))));
}

static void send_comm_stats(uint8_t result)
{
    pny_comm_stats_payload_t out;
    out.subcmd = PNY_DEBUG_COMM_STATS;
    out.result = result;
    /* No interrupt lock: a torn counter is harmless, a late edge is not. */
    out.settings = comm_settings;
    comm_diag.accel_krpm_s = accel_krpm_s();
    out.diag = comm_diag;
    comm_diag.innov_max = 0u;
    comm_diag.proc_max_us = 0u;
    send_frame(PNY_CMD_DEBUG, &out, sizeof(out));
}

/* 4-byte form (legacy tools) sets lead only; 11-byte form sets everything. */
static void handle_set_observer(const uint8_t *payload, uint8_t len)
{
    pny_comm_settings_t in = comm_settings;
    uint8_t result = PNY_RESULT_OK;
    if (len == sizeof(pny_observer_payload_t)) {
        memcpy(&in.lead_us, payload + 1, sizeof(in.lead_us));
    } else if (len == sizeof(pny_comm_settings_payload_t)) {
        memcpy(&in, payload + 1, sizeof(in));
    } else {
        result = PNY_RESULT_BAD_ARG;
    }
    if (result == PNY_RESULT_OK &&
        (in.lead_us < OBSERVER_LEAD_MIN_US || in.lead_us > OBSERVER_LEAD_MAX_US ||
         in.alpha_shift > 8u || in.beta_shift > 16u || in.gamma_shift > 24u ||
         in.fw_us < -500 || in.fw_us > 500 || in.fw_max_deg < 0 || in.fw_max_deg > 90)) {
        result = PNY_RESULT_RANGE;
    }
    if (result == PNY_RESULT_OK) {
        irq_lock();
        comm_settings = in;
        fw_max_turn16 = lead_deg_to_turn16(in.fw_max_deg);
        nvic_set_priority(NVIC_I2C1_IRQ, (in.flags & PNY_COMM_FLAG_I2C_FIRST) != 0u ? 0x00 : 0x40);
        nvic_set_priority(NVIC_TIM21_IRQ, (in.flags & PNY_COMM_FLAG_I2C_FIRST) != 0u ? 0x40 : 0x00);
        irq_unlock();
    }
    send_comm_stats(result);
}

static void handle_cal_read_blob(const uint8_t *payload, uint8_t len)
{
    uint8_t count = len == 4u ? payload[3] : 0u;
    uint16_t offset = len == 4u ? (uint16_t)(payload[1] | ((uint16_t)payload[2] << 8)) : 0u;
    const uint8_t *blob = (const uint8_t *)pennyesc_calibration_active();
    uint8_t *reply = (uint8_t *)&uart_state;
    reply[0] = PNY_RESULT_BAD_ARG;
    if (count == 0u || count >= PNY_FRAME_MAX_PAYLOAD || offset + count > sizeof(pennyesc_calibration_blob_t)) {
        count = 0u;
    } else if (!blob) {
        reply[0] = PNY_RESULT_NOT_CALIBRATED;
        count = 0u;
    } else {
        reply[0] = PNY_RESULT_OK;
        memcpy(reply + 1, blob + offset, count);
    }
    send_frame(PNY_CMD_CAL, reply, (uint8_t)(count + 1u));
}

static __attribute__((noinline)) void handle_cal(const uint8_t *payload, uint8_t len)
{
    if (len == 0u) {
        pny_cal_status_payload_t out = {PNY_RESULT_BAD_ARG, 0, 0, 0};
        send_frame(PNY_CMD_CAL, &out, sizeof(out));
        return;
    }

    switch (payload[0]) {
    case PNY_CAL_READ_BLOB:
        handle_cal_read_blob(payload, len);
        break;
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

static __attribute__((noinline)) void handle_debug(const uint8_t *payload, uint8_t len)
{
    if (len == 0u) {
        send_capture_status(PNY_RESULT_BAD_ARG);
        return;
    }

    switch (payload[0]) {
    case PNY_DEBUG_CAPTURE_START:
        send_capture_status(capture_start(payload, len));
        break;
    case PNY_DEBUG_CAPTURE_STATUS:
        if (len == 2u && payload[1] == 1u && capture_state.active) {
            irq_lock();
            capture_finish();
            irq_unlock();
        }
        send_capture_status(PNY_RESULT_OK);
        break;
    case PNY_DEBUG_COMM_STATS:
        send_comm_stats(PNY_RESULT_OK);
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

static __attribute__((noinline)) void process_frame(uint8_t header, const uint8_t *payload, uint8_t len)
{
    uint8_t address = header >> 4;
    uint8_t cmd = header & 0x0Fu;

    if (address != ESC_ADDRESS) {
        return;
    }
    switch (cmd) {
    case PNY_CMD_GET_STATUS:
        handle_get_status();
        break;
    case PNY_CMD_ENTER_BOOT:
        handle_enter_boot(payload, len);
        break;
    case PNY_CMD_GET_POS_VEL:
    {
        pny_pos_vel_payload_t out;
        out.position_turn32 = position_from_zero(absolute_position_turn32);
        out.velocity_turn32_per_s = velocity_now();
        send_frame(PNY_CMD_GET_POS_VEL, &out, sizeof(out));
        break;
    }
    case PNY_CMD_ZERO_POSITION:
        handle_zero_position(payload, len);
        break;
    case PNY_CMD_BRAKE:
        handle_brake(payload, len);
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

        const uint8_t *frame;
        uint8_t frame_len;
        if (pny_frame_parser_push(&frame_parser, byte, system_millis, UART_FRAME_TIMEOUT_MS, &frame, &frame_len)) {
            if ((frame[1] >> 4) == ESC_ADDRESS) {
                uart_last_frame_ms = system_millis;
            }
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
    /* Record why we reset: brownout from regenerative spikes shows as POR/PIN. */
    comm_diag.reset_flags = (uint8_t)(RCC_CSR >> 24);
    RCC_CSR |= RCC_CSR_RMVF;
    comm_diag.tick_us = SENSOR_TICK_US;
    clock_setup();
    systick_setup();
    __asm__("cpsie i");
    usart2_setup(UART_APP_BAUD);
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
        refresh_sensor_xyz(encoder_angle_valid());
    }

    while (1) {
        if (current_mode == PNY_MODE_IDLE && comm_scheduler.active) {
            comm_scheduler_stop();
        }
        uart_poll();
        if (current_mode == PNY_MODE_CAL) {
            calibration_poll();
        }

        mct_idle_check();
        control_restart_poll();
        if (pending_reset_ms != 0u && (int32_t)(system_millis - pending_reset_ms) >= 0) {
            scb_reset_system();
        }
    }
}
