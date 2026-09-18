/* Host fixture. test_firmware.py inserts production functions below. */
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pennyesc_timing.h"
#define PNY_DRIVE_BRUSHED 0
#define PNY_MODE_RUN 1
#define POLE_PAIRS 6
#define REVERSE_HALL_PHASE_TURN16 32768
#define SENSOR_STALE_US 10000u
#define TIM21 0
#define NVIC_TIM21_IRQ 0
#define TIM_DIER_CC2IE 4u
#define TIM_SR_CC2IF 4u
#define TIM_EGR_CC2G 4u
static uint32_t dier, sr, ccr;
#define TIM_DIER(t) dier
#define TIM_SR(t) sr
#define TIM_CCR2(t) ccr
/* STATE */
static comm_scheduler_t comm_scheduler;
static struct { bool active; } capture_state;
static struct { int32_t position_turn32; } observer_state;
static const uint16_t comm_sector_start_turn16[] = {0, 10923, 21846, 32768, 43691, 54614, 0};
static int32_t observer_velocity_remainder;
static int32_t comm_velocity_turn32_per_s, current_lead_turn16, observer_lead_us;
static int current_mode, current_duty, sensor_ready, isr_overrun_count;
static uint16_t clock_tick;
static int hall, edges, stopped, software_events;
static unsigned preempt_on_mask;
static uint16_t deadline_after_preemption;
static unsigned elapsed_us, calculation_us, edge_error;
static int rotor_direction;
static bool measuring_edge;
void tim21_isr(void);
static double rotor(double seconds);
static void advance_time(unsigned us);
static uint16_t sched_now_us(void) { return clock_tick; }
static void timer_clear_flag(int t, unsigned flag) { (void)t; sr &= ~flag; }
static void timer_generate_event(int t, unsigned event) {
    (void)t; (void)event; sr |= TIM_SR_CC2IF; software_events++;
}
static void nvic_disable_irq(int irq) {
    (void)irq;
    if (calculation_us) advance_time(calculation_us);
    while (preempt_on_mask) {
        preempt_on_mask--;
        clock_tick = comm_scheduler.next_comm_tick + 2;
        sr |= TIM_SR_CC2IF;
        tim21_isr();
        deadline_after_preemption = comm_scheduler.next_comm_tick;
    }
}
static void nvic_enable_irq(int irq) { (void)irq; }
static int32_t pennyesc_calibration_commutation_alignment_turn16(void) { return 0; }
static void set_hall_outputs_raw(uint8_t sector) {
    hall = sector; edges++;
    if (measuring_edge && elapsed_us > 10000) {
        uint16_t actual = (uint16_t)((int64_t)(rotor_direction * rotor(elapsed_us / 1000000.0)) * POLE_PAIRS +
                                    rotor_direction * 16384);
        uint16_t boundary = rotor_direction > 0 ? comm_sector_start_turn16[sector] :
            (uint16_t)(comm_sector_start_turn16[sector_next(sector, 1)] - 1u);
        unsigned error = abs((int16_t)(boundary - actual));
        if (error > edge_error) edge_error = error;
    }
}
static void clear_motor_outputs(void) { current_duty = 0; hall = -1; }
static void control_disable(void) { current_duty = 0; }
static void comm_stop_from_isr(void) { current_mode = 0; dier = 0; stopped++; }
/* FUNCTIONS */

static void advance_time(unsigned us) {
    while (us--) {
        elapsed_us++;
        clock_tick = elapsed_us;
        if ((dier & TIM_DIER_CC2IE) && tick_due(clock_tick, comm_scheduler.next_comm_tick + 2)) {
            sr |= TIM_SR_CC2IF;
            measuring_edge = true;
            tim21_isr();
            measuring_edge = false;
        }
    }
}

static void reset(void) {
    memset(&comm_scheduler, 0, sizeof(comm_scheduler));
    comm_scheduler.sector = 0xff;
    dier = sr = ccr = 0;
    current_mode = PNY_MODE_RUN;
    current_duty = 100;
    sensor_ready = 1;
    capture_state.active = true;
    edges = stopped = software_events = isr_overrun_count = 0;
    observer_lead_us = 0;
    current_lead_turn16 = 0;
    clock_tick = 65000;
    observer_state.position_turn32 = 1234;
    comm_scheduler.position_tick = clock_tick;
}

static void spin(unsigned rpm, int direction) {
    reset();
    comm_velocity_turn32_per_s = direction * (int32_t)((uint64_t)rpm * 65536 / 60);
    comm_scheduler_schedule_sector_event(clock_tick, direction);
    assert(dier & TIM_DIER_CC2IE);
    double ideal_period = 60000000.0 / (rpm * 36.0);
    assert(fabs(comm_scheduler.period_q8 / 256.0 - ideal_period) < 0.02);
    uint16_t previous = comm_scheduler.next_comm_tick;
    unsigned elapsed = 0;
    for (unsigned i = 0; i < 10000; ++i) {
        uint16_t deadline = comm_scheduler.next_comm_tick;
        // ISR latency varies 0..3 us and crosses the 16-bit timer wrap repeatedly.
        clock_tick = deadline + (i % 4);
        comm_scheduler.position_tick = clock_tick;
        sr |= TIM_SR_CC2IF;
        int expected = sector_next(hall, direction);
        tim21_isr();
        assert(!stopped && hall == expected);
        if (i) elapsed += (uint16_t)(deadline - previous);
        previous = deadline;
    }
    assert(fabs(elapsed - ideal_period * 9999) < 200.0);
    assert(isr_overrun_count == 0);
}

static double ramp_seconds = 1.0;

static double rotor(double seconds) {
    // Accelerate 0 -> 50,000 rpm in one second, then hold.
    double turns = seconds < ramp_seconds ? seconds * seconds / (2 * ramp_seconds) : seconds - ramp_seconds / 2;
    return turns * (50000.0 / 60.0) * 65536;
}

static void observer_ramp(int direction, unsigned lead_us, unsigned alpha, unsigned beta) {
    reset();
    comm_scheduler.position_tick = 0;
    observer_state.position_turn32 = 0;
    comm_velocity_turn32_per_s = observer_velocity_remainder = 0;
    observer_lead_us = lead_us;
    for (unsigned i = 2; i <= 18000; ++i) {
        unsigned tick = i * 100;
        int noise = ((int)i % 3 - 1) * 4;
        uint16_t measured = (uint16_t)((int64_t)(direction * rotor((tick - lead_us) / 1000000.0)) + noise);
        observer_ab_update(measured, (uint16_t)tick, alpha, beta);
        comm_scheduler.position_tick = (uint16_t)tick;
        uint16_t predicted = (uint16_t)observer_position_at_tick((uint16_t)(tick + 100));
        uint16_t actual = (uint16_t)(int64_t)(direction * rotor((tick + 100) / 1000000.0));
        int error = abs((int16_t)(predicted - actual)) * POLE_PAIRS;
        if (i > 300) assert(error < (ramp_seconds < 0.1 ? 3600 : 1200)); // Fast ramp: <20 electrical degrees.
        if (i > 12000) assert(error < 150); // <0.83 degrees at steady speed.
    }
}

static void scheduled_ramp(int direction, unsigned work_us) {
    reset();
    elapsed_us = edge_error = 0;
    rotor_direction = direction;
    clock_tick = 0;
    comm_scheduler.position_tick = 0;
    observer_state.position_turn32 = 0;
    comm_velocity_turn32_per_s = observer_velocity_remainder = 0;
    current_lead_turn16 = 16384;
    observer_lead_us = 120;
    ramp_seconds = 0.05;
    while (elapsed_us < 100000) {
        advance_time(1);
        if (elapsed_us >= 300 && elapsed_us % 100 == 0) {
            uint16_t sample_tick = clock_tick - 100;
            int noise = ((int)(elapsed_us / 100) % 3 - 1) * 40;
            uint16_t measured = (uint16_t)((int64_t)(direction * rotor((elapsed_us - 220) / 1000000.0)) + noise);
            observer_ab_update(measured, sample_tick, 4, 32);
            comm_scheduler.position_tick = sample_tick;
            calculation_us = work_us;
            comm_scheduler_schedule_sector_event(clock_tick, direction);
            calculation_us = 0;
            assert(!stopped);
        }
    }
    assert(edge_error < 5462); // Actual sector edges stay within 30 electrical degrees.
}

int main(void) {
    assert(abs_u32(INT32_MIN) == 2147483648u);
    assert(position_target_direction(INT32_MAX, INT32_MIN, 100) == 1);
    assert(position_target_direction(INT32_MIN, INT32_MAX, 100) == -1);
    assert(position_target_direction(INT32_MAX, INT32_MAX - 99, 100) == 0);
    assert(position_target_direction(INT32_MIN, INT32_MIN + 99, 100) == 0);
    assert(position_target_direction(0, 0, 100) == 0);
    for (unsigned lead = 90; lead <= 180; lead += 30) {
        observer_ramp(1, lead, 8, 128); observer_ramp(-1, lead, 8, 128);
        observer_ramp(1, lead, 4, 32); observer_ramp(-1, lead, 4, 32);
    }
    ramp_seconds = 0.05; // 1,000,000 rpm/s, matching the bench acceleration scale.
    observer_ramp(1, 120, 4, 32); observer_ramp(-1, 120, 4, 32);
    spin(17000, 1); spin(17000, -1);
    spin(50000, 1); spin(50000, -1);
    for (unsigned work = 0; work <= 60; work += 20) {
        scheduled_ramp(1, work); scheduled_ramp(-1, work);
    }
    assert(tick_due(2, 65535) && !tick_due(65535, 2));
    assert(sector_period_q8(0) == 0);
    for (int sign = -1; sign <= 1; sign += 2) {
        int32_t v = sign * 54613333;
        for (int us = -1000; us <= 10000; us += 7) {
            double ideal = (double)v * us / 1000000;
            assert(fabs(position_delta(v, us) - ideal) < 6);
        }
    }
    // A compare already elapsed while software was calculating/programming it.
    reset(); clock_tick = 4; comm_arm(65535);
    assert((sr & TIM_SR_CC2IF) && software_events == 1);
    // Clear the previous event before arming a future edge.
    comm_arm(10); assert(!(sr & TIM_SR_CC2IF));
    assert(ccr == 10 && (dier & TIM_DIER_CC2IE));
    // Sensor updates preserve an imminent edge and bound phase corrections.
    assert(sector_correction(2, 40, 33u << 8) == 8);
    assert(sector_correction(10, 90, 40u << 8) == 10);
    assert(sector_correction(10, 65400, 40u << 8) == -10);
    reset(); comm_velocity_turn32_per_s = 54613333;
    comm_scheduler_schedule_sector_event(clock_tick, 1);
    uint8_t next = comm_scheduler.next_sector;
    uint16_t edge = comm_scheduler.next_comm_tick;
    clock_tick++;
    observer_state.position_turn32 += 1000;
    comm_scheduler_schedule_sector_event(clock_tick, 1);
    assert(comm_scheduler.next_sector == next);
    assert(abs((int16_t)(comm_scheduler.next_comm_tick - edge)) <= 8);
    // A sector ISR can fire while the sensor correction is being calculated.
    // Carry the correction forward to the new upcoming edge rather than lose it.
    for (int direction = -1; direction <= 1; direction += 2) {
        for (unsigned count = 1; count <= 6; count++) {
            reset(); comm_velocity_turn32_per_s = direction * 54613333;
            comm_scheduler.edge_count = 253; // Exercise the event counter wrap.
            comm_scheduler_schedule_sector_event(clock_tick, direction);
            observer_state.position_turn32 += direction * 600;
            preempt_on_mask = count;
            comm_scheduler_schedule_sector_event(clock_tick, direction);
            assert(!stopped && comm_scheduler.next_sector == sector_next(hall, direction));
            int correction = (int16_t)(comm_scheduler.next_comm_tick - deadline_after_preemption);
            assert(correction <= -4 && correction >= -8);
        }
    }
    // A missed full sector stops output; it must not race through skipped sectors.
    clock_tick = comm_scheduler.next_comm_tick + (comm_scheduler.period_q8 >> 8);
    sr |= TIM_SR_CC2IF; tim21_isr();
    assert(stopped == 1 && current_duty == 0 && !sensor_ready && !capture_state.active);
    // Loss of measurements stops even when compare timing is otherwise healthy.
    reset(); comm_velocity_turn32_per_s = 54613333;
    comm_scheduler_schedule_sector_event(clock_tick, 1);
    clock_tick = comm_scheduler.next_comm_tick;
    comm_scheduler.position_tick = clock_tick - SENSOR_STALE_US - 1;
    sr |= TIM_SR_CC2IF; tim21_isr(); assert(stopped == 1);
    // An impossible speed estimate faults instead of pulsing PWM each control tick.
    reset(); comm_velocity_turn32_per_s = INT32_MAX;
    comm_scheduler_schedule_sector_event(clock_tick, 1);
    assert(stopped == 1 && current_duty == 0);
    return 0;
}
