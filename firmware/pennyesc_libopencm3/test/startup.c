/* SysTick keeps time; TIM21 CC1 paces sensor reads; PendSV runs control. */
#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include "pennyesc_protocol.h"
#define SENSOR_TICK_US 125u
#define SENSOR_STALE_US 10000u
#define TIM21 21
#define TIM_DIER_CC1IE 2u
#define TIM_DIER_CC2IE 4u
#define TIM_SR_CC1IF 2u
#define TIM_SR_CC2IF 4u
#define STK_CSR_CLKSOURCE_AHB 1
#define NVIC_SYSTICK_IRQ -1
#define NVIC_PENDSV_IRQ -2
static uint32_t dier, ccr1, cleared;
#define TIM_DIER(timer) dier
#define TIM_CCR1(timer) ccr1
/* STATE */
static comm_scheduler_t comm_scheduler;
static pny_comm_diag_t comm_diag;
static uint32_t system_millis, isr_overrun_count;
static uint16_t isr_duration_us, isr_max_us, now, work_us = 20;
static uint8_t current_mode;
static unsigned reload, control_calls, sensor_cancels, systick_priority, pendsv_priority;
static bool tick_enabled;
static void systick_set_clocksource(int clock) { assert(clock == STK_CSR_CLKSOURCE_AHB); }
static void systick_set_reload(unsigned value) { reload = value; }
static void nvic_set_priority(int irq, unsigned priority) {
    if (irq == NVIC_SYSTICK_IRQ) systick_priority = priority;
    else if (irq == NVIC_PENDSV_IRQ) pendsv_priority = priority;
    else assert(0);
}
static void systick_interrupt_enable(void) { tick_enabled = true; }
static void systick_counter_enable(void) {}
static uint16_t sched_now_us(void) { return now; }
static void timer_clear_flag(int timer, unsigned flag) { assert(timer == TIM21); cleared |= flag; }
static void tmag5273_async_cancel(void) { sensor_cancels++; }
static void comm_sensor_tick(void) { control_calls++; now += work_us; }
/* FUNCTIONS */
int main(void) {
    systick_setup();
    assert(tick_enabled && reload == 31999u);   // 1 ms
    assert(pendsv_priority == 0x80 && systick_priority == 0xC0);
    for (unsigned i = 0; i < 5; ++i) sys_tick_handler();
    assert(system_millis == 5);

    current_mode = PNY_MODE_RUN;
    pend_sv_handler();   // RUN can be set before the scheduler starts.
    assert(control_calls == 0);
    now = 65530;
    comm_scheduler_start();
    assert(comm_scheduler.active && (dier & TIM_DIER_CC1IE) && !(dier & TIM_DIER_CC2IE));
    assert(ccr1 == (uint16_t)(65530 + 10) && (cleared & (TIM_SR_CC1IF | TIM_SR_CC2IF)));
    assert(comm_scheduler.position_tick == now && comm_scheduler.sample_limit_us == SENSOR_STALE_US);
    pend_sv_handler();
    assert(control_calls == 1 && isr_duration_us == 20 && isr_max_us == 20);
    assert(comm_diag.proc_max_us == 20 && isr_overrun_count == 0);
    work_us = SENSOR_TICK_US;
    pend_sv_handler();
    assert(isr_overrun_count == 1 && comm_diag.proc_max_us == SENSOR_TICK_US);
    comm_scheduler_stop();
    assert(!comm_scheduler.active && sensor_cancels == 1 && !(dier & (TIM_DIER_CC1IE | TIM_DIER_CC2IE)));
    pend_sv_handler();
    assert(control_calls == 2);
    system_millis = UINT32_MAX;
    sys_tick_handler();
    assert(system_millis == 0); // Stopping the motor never stops application time.
    return 0;
}
