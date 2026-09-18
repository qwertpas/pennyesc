/* The real SysTick handler must dispatch control after scheduler startup. */
#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#define SENSOR_TICK_US 100u
#define SENSOR_STALE_US 10000u
#define PNY_MODE_RUN 1
#define TIM21 21
#define TIM_DIER_CC2IE 4u
#define TIM_SR_CC2IF 4u
#define STK_CSR_CLKSOURCE_AHB 1
#define NVIC_SYSTICK_IRQ -1
static uint32_t dier;
#define TIM_DIER(timer) dier
/* STATE */
static comm_scheduler_t comm_scheduler;
static uint32_t system_millis, isr_overrun_count;
static uint16_t isr_duration_us, isr_max_us, now, work_us = 20;
static uint8_t current_mode;
static unsigned reload, control_calls, sensor_starts, sensor_cancels;
static bool tick_enabled;
static void systick_set_clocksource(int clock) { assert(clock == STK_CSR_CLKSOURCE_AHB); }
static void systick_set_reload(unsigned value) { reload = value; }
static void nvic_set_priority(int irq, unsigned priority) {
    assert(irq == NVIC_SYSTICK_IRQ && priority == 0x80);
}
static void systick_interrupt_enable(void) { tick_enabled = true; }
static void systick_counter_enable(void) {}
static uint16_t sched_now_us(void) { return now; }
static void timer_clear_flag(int timer, unsigned flag) {
    assert(timer == TIM21 && flag == TIM_SR_CC2IF);
}
static bool tmag5273_async_start_xy(uint16_t tick) {
    assert(tick == now); sensor_starts++; return true;
}
static void tmag5273_async_cancel(void) { sensor_cancels++; }
static void comm_sensor_tick(void) { control_calls++; now += work_us; }
/* FUNCTIONS */
int main(void) {
    systick_setup();
    assert(tick_enabled && reload == 3199); // 32 MHz / 3200 = 10 kHz.
    for (unsigned i = 0; i < 10; ++i) sys_tick_handler();
    assert(system_millis == 1 && control_calls == 0);
    current_mode = PNY_MODE_RUN;
    sys_tick_handler(); // RUN can be set before scheduler initialization finishes.
    assert(control_calls == 0);
    now = 65500;
    comm_scheduler_start();
    assert(comm_scheduler.active && sensor_starts == 1);
    assert(comm_scheduler.position_tick == now && comm_scheduler.sample_limit_us == SENSOR_STALE_US);
    sys_tick_handler();
    assert(control_calls == 1 && isr_duration_us == 20 && isr_max_us == 20);
    for (unsigned i = 0; i < 8; ++i) sys_tick_handler();
    assert(system_millis == 2 && control_calls == 9);
    work_us = SENSOR_TICK_US;
    sys_tick_handler();
    assert(isr_overrun_count == 1);
    comm_scheduler_stop();
    assert(!comm_scheduler.active && sensor_cancels == 1 && tick_enabled);
    for (unsigned i = 0; i < 9; ++i) sys_tick_handler();
    assert(system_millis == 3 && control_calls == 10);
    system_millis = UINT32_MAX;
    for (unsigned i = 0; i < 10; ++i) sys_tick_handler();
    assert(system_millis == 0); // Stopping the motor must never stop application time.
    return 0;
}
