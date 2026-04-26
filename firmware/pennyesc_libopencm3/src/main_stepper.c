#include <libopencm3/cm3/systick.h>
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
#include "pennyesc_protocol.h"
#include "tmag5273.h"

#ifndef PNY_ESC_ADDRESS
#define PNY_ESC_ADDRESS 1
#endif

#define ESC_ADDRESS PNY_ESC_ADDRESS

#if ESC_ADDRESS > 0xF
#error "ESC_ADDRESS must fit in 4 bits"
#endif

#define PWM_PERIOD 799
#define DUTY_LIMIT PWM_PERIOD
#define UART_APP_BAUD 230400u
#define SENSOR_POLL_MS 5u
#define UART_FRAME_TIMEOUT_MS 10u
#define FRAME_BUF_SIZE (PNY_FRAME_MAX_PAYLOAD + 4u)
#define WALK_DUTY 0
#define WALK_STEP_MS 200u
#define WALK_REVERSE_MS 20000u
#define MCT_DEBUG_REG_COUNT 13u

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

static const uint8_t hall_pattern[6][3] = {
    {1, 1, 0},
    {1, 0, 0},
    {1, 0, 1},
    {0, 0, 1},
    {0, 1, 1},
    {0, 1, 0},
};

static volatile uint32_t system_millis;
static volatile int16_t sensor_x;
static volatile int16_t sensor_y;
static volatile int16_t sensor_z;
static volatile bool sensor_ready;
static volatile int16_t current_duty;
static volatile int8_t current_step = -1;
static volatile int32_t transition_count;
volatile uint16_t mct_debug_regs[MCT_DEBUG_REG_COUNT] __attribute__((used));
volatile uint32_t mct_debug_reads __attribute__((used));
volatile uint32_t mct_debug_last_ms __attribute__((used));
volatile uint32_t spi_debug_cr1 __attribute__((used));
volatile uint32_t spi_debug_cr2 __attribute__((used));
volatile uint32_t spi_debug_sr __attribute__((used));

static uint8_t frame_buf[FRAME_BUF_SIZE];
static uint8_t frame_idx;
static uint8_t frame_expected;
static uint32_t frame_last_byte_ms;
static uint32_t last_sensor_poll_ms;
static uint32_t last_walk_step_ms;
static uint32_t last_walk_reverse_ms;
static int8_t walk_direction = 1;
static bool auto_walk = true;

void sys_tick_handler(void);

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
    systick_set_reload(31999);
    systick_interrupt_enable();
    systick_counter_enable();
    __asm__("cpsie i");
}

static void delay_ms(uint32_t ms)
{
    uint32_t start = system_millis;
    while ((system_millis - start) < ms) {
    }
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

static uint8_t frame_header(uint8_t cmd)
{
    return (uint8_t)((ESC_ADDRESS << 4) | (cmd & 0x0Fu));
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

static void set_hall_outputs(uint8_t step)
{
    const uint8_t *pattern = hall_pattern[step % 6u];

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

static bool step_valid(int8_t step)
{
    return step >= -1 && step <= 5;
}

static void apply_current_step(void)
{
    apply_pwm_duty(current_duty);
    if (current_step < 0) {
        clear_hall_outputs();
        return;
    }
    set_hall_outputs((uint8_t)current_step);
}

static void set_step_direct(int8_t step)
{
    bool changed = (step != current_step);
    current_step = step;
    apply_current_step();
    if (changed) {
        transition_count++;
    }
}

static void set_step_with_blank(int8_t step, uint16_t blank_ms)
{
    bool changed = (step != current_step);
    apply_pwm_duty(0);
    clear_hall_outputs();
    if (blank_ms != 0u) {
        delay_ms(blank_ms);
    }
    current_step = step;
    apply_current_step();
    if (changed) {
        transition_count++;
    }
}

static void refresh_sensor_xyz(void)
{
    int16_t x;
    int16_t y;
    int16_t z;

    if (!tmag5273_read_xyz_fast(&x, &y, &z)) {
        sensor_ready = false;
        sensor_x = 0;
        sensor_y = 0;
        sensor_z = 0;
        return;
    }

    sensor_ready = true;
    sensor_x = x;
    sensor_y = y;
    sensor_z = z;
}

static void refresh_mct_debug_regs(void)
{
    for (uint8_t reg = 0u; reg < MCT_DEBUG_REG_COUNT; reg++) {
        mct_debug_regs[reg] = mct8316z_read_reg(reg);
    }

    spi_debug_cr1 = SPI_CR1(SPI1);
    spi_debug_cr2 = SPI_CR2(SPI1);
    spi_debug_sr = SPI_SR(SPI1);
    mct_debug_reads++;
    mct_debug_last_ms = system_millis;
}

static void setup_mct_driver(void)
{
    mct8316z_set_pwm_mode_sync_dig();
    mct8316z_set_hall_hys_high();
    mct8316z_set_direction(false);
    mct8316z_disable_protections();
}

static void reset_mct_driver(void)
{
    apply_pwm_duty(0);
    delay_ms(2u);
    setup_mct_driver();
    refresh_mct_debug_regs();
}

static void auto_walk_poll(void)
{
    if (!auto_walk) {
        return;
    }

    uint32_t now = system_millis;
    if ((now - last_walk_reverse_ms) >= WALK_REVERSE_MS) {
        walk_direction = (int8_t)-walk_direction;
        last_walk_reverse_ms = now;
    }

    if ((now - last_walk_step_ms) < WALK_STEP_MS) {
        return;
    }

    last_walk_step_ms = now;
    if (current_step < 0) {
        current_step = (walk_direction > 0) ? 0 : 5;
        apply_current_step();
        transition_count++;
        return;
    }

    current_step = (int8_t)((current_step + walk_direction + 6) % 6);
    apply_current_step();
    transition_count++;
}

static void fill_status_payload(pny_status_payload_t *payload, uint8_t result)
{
    payload->result = result;
    payload->mode = (current_step >= 0) ? PNY_MODE_RUN : PNY_MODE_IDLE;
    payload->flags = sensor_ready ? PNY_FLAG_SENSOR_OK : 0u;
    payload->faults = sensor_ready ? 0u : PNY_FAULT_SENSOR;
    payload->x = sensor_x;
    payload->y = sensor_y;
    payload->z = sensor_z;
    payload->angle_turn16 = (current_step < 0) ? 0u : (uint16_t)((uint32_t)current_step * 65536u / 6u);
    payload->position_crad = current_step;
    payload->velocity_crads = transition_count;
    payload->duty = current_duty;
}

static void send_status_response(uint8_t cmd, uint8_t result)
{
    pny_status_payload_t payload;
    fill_status_payload(&payload, result);
    send_frame(cmd, &payload, sizeof(payload));
}

static void parser_reset(void)
{
    frame_idx = 0u;
    frame_expected = 0u;
}

static void handle_set_duty(const uint8_t *payload, uint8_t len)
{
    int16_t duty;

    if (len != 2u) {
        send_status_response(PNY_CMD_SET_DUTY, PNY_RESULT_BAD_ARG);
        return;
    }

    memcpy(&duty, payload, sizeof(duty));
    auto_walk = false;
    reset_mct_driver();
    current_duty = duty;
    apply_current_step();
    send_status_response(PNY_CMD_SET_DUTY, PNY_RESULT_OK);
}

static void handle_step_set(const uint8_t *payload, uint8_t len)
{
    int8_t step;

    if (len != 1u) {
        send_status_response(PNY_CMD_STEP_SET, PNY_RESULT_BAD_ARG);
        return;
    }

    memcpy(&step, payload, sizeof(step));
    if (!step_valid(step)) {
        send_status_response(PNY_CMD_STEP_SET, PNY_RESULT_RANGE);
        return;
    }

    auto_walk = false;
    if (step == current_step && current_duty != 0) {
        reset_mct_driver();
    }
    set_step_direct(step);
    send_status_response(PNY_CMD_STEP_SET, PNY_RESULT_OK);
}

static void handle_step_transition(const uint8_t *payload, uint8_t len)
{
    int8_t step;
    uint16_t blank_ms;

    if (len != 3u) {
        send_status_response(PNY_CMD_STEP_TRANSITION, PNY_RESULT_BAD_ARG);
        return;
    }

    memcpy(&step, payload, sizeof(step));
    memcpy(&blank_ms, &payload[1], sizeof(blank_ms));
    if (!step_valid(step)) {
        send_status_response(PNY_CMD_STEP_TRANSITION, PNY_RESULT_RANGE);
        return;
    }

    auto_walk = false;
    set_step_with_blank(step, blank_ms);
    send_status_response(PNY_CMD_STEP_TRANSITION, PNY_RESULT_OK);
}

static void process_frame(uint8_t header, const uint8_t *payload, uint8_t len)
{
    uint8_t address = header >> 4;
    uint8_t cmd = header & 0x0Fu;

    if (address != ESC_ADDRESS) {
        return;
    }

    switch (cmd) {
    case PNY_CMD_GET_STATUS:
        send_status_response(PNY_CMD_GET_STATUS, PNY_RESULT_OK);
        break;
    case PNY_CMD_SET_DUTY:
        handle_set_duty(payload, len);
        break;
    case PNY_CMD_STEP_SET:
        handle_step_set(payload, len);
        break;
    case PNY_CMD_STEP_TRANSITION:
        handle_step_transition(payload, len);
        break;
    default:
        break;
    }
}

static void uart_poll(void)
{
    while ((USART_ISR(USART2) & USART_ISR_RXNE) != 0u) {
        uint8_t byte = (uint8_t)USART_RDR(USART2);
        frame_last_byte_ms = system_millis;

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
    }

    if (frame_idx != 0u && (system_millis - frame_last_byte_ms) > UART_FRAME_TIMEOUT_MS) {
        parser_reset();
    }
}

int main(void)
{
    clock_setup();
    systick_setup();
    usart2_setup(UART_APP_BAUD);
    i2c1_setup();
    tim2_pwm_setup();
    motor_gpio_setup();

    mct8316z_init();
    delay_ms(5u);
    setup_mct_driver();
    refresh_mct_debug_regs();
    current_duty = WALK_DUTY;
    current_step = 0;
    apply_current_step();
    transition_count = 1;
    last_walk_step_ms = system_millis;
    last_walk_reverse_ms = system_millis;

    sensor_ready = tmag5273_init();
    if (sensor_ready) {
        tmag5273_clear_por();
        refresh_sensor_xyz();
    }

    while (1) {
        uart_poll();
        if ((system_millis - last_sensor_poll_ms) >= SENSOR_POLL_MS) {
            last_sensor_poll_ms = system_millis;
            refresh_sensor_xyz();
        }
        if (current_duty == 0 && (system_millis - mct_debug_last_ms) >= 1000u) {
            refresh_mct_debug_regs();
        }
        auto_walk_poll();
    }
}
