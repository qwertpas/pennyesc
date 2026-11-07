/* Host fixture. test_firmware.py inserts production state and functions. */
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "pennyesc_protocol.h"
#include "pennyesc_timing.h"
#define PNY_DRIVE_BRUSHED 0
#define POLE_PAIRS 6
#define REVERSE_HALL_PHASE_TURN16 32768
#define SENSOR_TICK_US 125u
#define SENSOR_STALE_US 10000u
#define TIM21 0
#define NVIC_TIM21_IRQ 0
#define TIM_DIER_CC2IE 4u
#define TIM_SR_CC1IF 2u
#define TIM_SR_CC2IF 4u
#define TIM_EGR_CC2G 4u
static uint32_t dier, sr, ccr;
#define TIM_DIER(t) dier
#define TIM_SR(t) sr
#define TIM_CCR2(t) ccr
/* DEFINES */
/* STATE */
static comm_scheduler_t comm_scheduler;
static observer_state_t observer_state;
static struct { bool active; } capture_state;
static const uint16_t comm_sector_start_turn16[] = {0, 10923, 21846, 32768, 43691, 54614, 0};
static pny_comm_settings_t comm_settings = {0, PNY_COMM_FLAG_GUARD, 3, 6, 12, 0, 35};
#define observer_lead_us comm_settings.lead_us
static pny_comm_diag_t comm_diag;
static int32_t fw_max_turn16 = 6372, current_lead_turn16;
static uint16_t guard_ticks;
static int current_mode, current_duty, sensor_ready, isr_overrun_count;
static int8_t current_direction, command_direction;
static int16_t command_duty, applied_duty;
static bool report_speed;
static int control_kp_q8, control_kd_q8, control_kv_q8, control_kf;
static uint16_t clock_tick;
static int hall, edges, stopped, software_events, captures_finished;
static unsigned preempt_on_mask;
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
    }
}
static void nvic_enable_irq(int irq) { (void)irq; }
static int32_t pennyesc_calibration_commutation_alignment_turn16(void) { return 0; }
static void set_hall_outputs_raw(uint8_t sector) {
    hall = sector; edges++;
    if (measuring_edge && elapsed_us > 20000) {
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
static void capture_trigger(void) {}
static void capture_finish(void) { captures_finished++; capture_state.active = false; }
static void sensor_tick_isr(void) {}
static void comm_scheduler_stop(void) { comm_scheduler.active = false; dier = 0; stopped++; }
static void get_drive_command(int16_t *duty, int8_t *direction) {
    *duty = command_duty; *direction = command_direction;
}
static void apply_pwm_duty(int16_t duty) { applied_duty = duty; }
static void comm_stop_from_isr(void) { current_mode = 0; dier = 0; stopped++; }
/* FUNCTIONS */

#define TURN32_PER_TURN16 65536.0
#define TICK_S (SENSOR_TICK_US * 1e-6)

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
    memset(&observer_state, 0, sizeof(observer_state));
    comm_scheduler.sector = 0xff;
    dier = sr = ccr = 0;
    current_mode = PNY_MODE_RUN;
    current_duty = 100;
    sensor_ready = 1;
    capture_state.active = true;
    edges = stopped = software_events = isr_overrun_count = captures_finished = 0;
    comm_settings.lead_us = 0;
    comm_settings.fw_us = 0;
    guard_ticks = 0;
    current_lead_turn16 = 0;
    clock_tick = 65000;
    comm_scheduler.position_tick = clock_tick;
}

/* Observer state for a rotor at `turn16` turning at `rpm`, sampled now. */
static void set_observer(double turn16, double rpm, int direction) {
    observer_state.pos = (uint32_t)(int64_t)llround(turn16 * TURN32_PER_TURN16);
    observer_state.vel = (int32_t)llround(direction * rpm / 60.0 * TICK_S * 4294967296.0);
    observer_state.acc = 0;
    observer_state.init = 2;
}

/* Sector the observer puts the rotor in now (it assumes constant velocity). */
static uint8_t expected_sector(int direction) {
    double ticks = ((int16_t)(clock_tick - comm_scheduler.position_tick) + comm_settings.lead_us) /
                   (double)SENSOR_TICK_US;
    uint32_t mech = observer_state.pos + (uint32_t)(int64_t)llround(observer_state.vel * ticks);
    uint16_t phase = commutation_phase(mech, direction, 0);
    return (uint8_t)(((uint32_t)phase * 6u) >> 16);
}

static void spin(unsigned rpm, int direction) {
    reset();
    elapsed_us = 0;
    clock_tick = 0;
    double ideal_period = 60000000.0 / (rpm * 36.0);
    for (unsigned i = 0; i < 250000; i++) {
        if (i % SENSOR_TICK_US == 0) {
            set_observer(direction * (double)rpm / 60.0 * 65536.0 * i / 1e6, rpm, direction);
            comm_scheduler.position_tick = clock_tick;
            comm_scheduler_schedule_sector_event(clock_tick, direction);
            assert(fabs(comm_scheduler.period_q8 / 256.0 - ideal_period) < 0.05);
            assert(hall == expected_sector(direction));
        }
        advance_time(1);
        assert(!stopped);
    }
}

static double ramp_seconds = 1.0;
static bool first_order;   /* motor-like: acceleration fades near free speed */

static double rotor(double seconds) {
    // 0 -> 50,000 rpm (turn16 units): constant acceleration for ramp_seconds,
    // or first order with the same initial acceleration.
    double turns = first_order ? seconds - ramp_seconds * (1.0 - exp(-seconds / ramp_seconds)) :
        seconds < ramp_seconds ? seconds * seconds / (2 * ramp_seconds) : seconds - ramp_seconds / 2;
    return turns * (50000.0 / 60.0) * 65536;
}

/* Electrical degrees between the observer's extrapolation one tick ahead and
 * the rotor, while the rotor accelerates. A type-2 loop (alpha-beta) lags by
 * ~50 degrees per 1M rpm/s here; the acceleration state removes that. */
static double observer_ramp(int direction, double ramp, double settle_s) {
    reset();
    ramp_seconds = ramp;
    double worst = 0;
    for (unsigned i = 1; i * SENSOR_TICK_US < 1200000; ++i) {
        double t = i * TICK_S;
        int noise = ((int)i % 3 - 1) * 8;
        uint16_t measured = (uint16_t)((int64_t)(direction * rotor(t)) + noise);
        observer_update(measured, (uint16_t)(i * SENSOR_TICK_US));
        double h = 1.0;   /* ticks ahead */
        double predicted = observer_state.pos / TURN32_PER_TURN16 +
                           (observer_state.vel * h + 0.5 * observer_state.acc * h * h) / TURN32_PER_TURN16;
        double actual = direction * rotor(t + TICK_S);
        double error = fabs(remainder(predicted - actual, 65536.0)) * POLE_PAIRS * 360.0 / 65536.0;
        if (t > settle_s && fabs(t - ramp) > 0.01 && error > worst) worst = error;
    }
    return worst;
}

static void scheduled_ramp(int direction, unsigned work_us) {
    reset();
    elapsed_us = edge_error = 0;
    rotor_direction = direction;
    clock_tick = 0;
    comm_scheduler.position_tick = 0;
    current_lead_turn16 = 16384;
    ramp_seconds = 0.05;   /* 1,000,000 rpm/s initially */
    first_order = true;
    while (elapsed_us < 100000) {
        advance_time(1);
        if (elapsed_us >= 300 && elapsed_us % SENSOR_TICK_US == 0) {
            /* The sample was taken 100 us ago; the stamp says exactly when. */
            uint16_t stamp = clock_tick - 100;
            int noise = ((int)(elapsed_us / SENSOR_TICK_US) % 3 - 1) * 40;
            uint16_t measured = (uint16_t)((int64_t)(direction * rotor((elapsed_us - 100) / 1e6)) + noise);
            observer_update(measured, stamp);
            calculation_us = work_us;
            comm_scheduler_schedule_sector_event(clock_tick, direction);
            calculation_us = 0;
            assert(!stopped);
        }
    }
    first_order = false;
    assert(edge_error < 2731); // Sector edges stay within 15 electrical degrees.
}

int main(void) {
    reset();
    comm_scheduler.active = true;
    current_direction = 1;
    set_observer(1000, 1000, 1);
    command_direction = -1;
    command_duty = -80;
    comm_control_tick(clock_tick);
    assert(current_mode == PNY_MODE_RUN && comm_scheduler.active && stopped == 0);
    assert(current_direction == -1 && applied_duty == -80);
    assert(hall == expected_sector(-1));
    assert(abs_u32(INT32_MIN) == 2147483648u);
    assert(position_target_direction(INT32_MAX, INT32_MIN, 100) == 1);
    assert(position_target_direction(INT32_MIN, INT32_MAX, 100) == -1);
    assert(position_target_direction(INT32_MAX, INT32_MAX - 99, 100) == 0);
    assert(position_target_direction(INT32_MIN, INT32_MIN + 99, 100) == 0);
    assert(position_target_direction(0, 0, 100) == 0);

    for (int direction = -1; direction <= 1; direction += 2) {
        assert(observer_ramp(direction, 1.0, 0.05) < 3.0);   // 50,000 rpm/s
        assert(observer_ramp(direction, 0.05, 0.02) < 15.0); // 1,000,000 rpm/s (alpha-beta: ~50)
    }
    // Gaps: one or several missed samples keep the velocity estimate.
    reset(); ramp_seconds = 10.0;
    for (unsigned i = 1; i < 2000; ++i) {
        if (i % 50 == 0 || i % 97 == 0 || (i > 900 && i < 905)) continue;
        uint16_t measured = (uint16_t)(int64_t)rotor(i * TICK_S + 0.5);
        observer_update(measured, (uint16_t)(i * SENSOR_TICK_US));
    }
    double rpm = observer_state.vel / 4294967296.0 / TICK_S * 60.0;
    double true_rpm = (0.5 + 2000 * TICK_S) / 10.0 * 50000.0;
    assert(fabs(rpm - true_rpm) < 50.0);
    // A glitch sample is rejected once, then accepted if it persists.
    uint32_t before = observer_state.pos;
    uint16_t rejected = comm_diag.rejected;
    observer_update((uint16_t)((before >> 16) + 20000), (uint16_t)(2000 * SENSOR_TICK_US));
    assert(comm_diag.rejected == rejected + 1);

    spin(17000, 1); spin(17000, -1);
    spin(50000, 1); spin(50000, -1);
    spin(60000, 1); spin(60000, -1);
    for (unsigned work = 0; work <= 60; work += 20) {
        scheduled_ramp(1, work); scheduled_ramp(-1, work);
    }
    assert(tick_due(2, 65535) && !tick_due(65535, 2));

    // Field weakening adds advance in the drive direction, capped; the slip
    // guard removes it, and braking never field-weakens.
    for (int direction = -1; direction <= 1; direction += 2) {
        reset(); set_observer(0, 40000, direction);
        comm_scheduler_schedule_sector_event(clock_tick, direction);
        assert(comm_diag.fw_deg == 0);
        comm_settings.fw_us = 25;   // 4 kHz electrical * 25 us = 36 degrees
        comm_scheduler_schedule_sector_event(clock_tick, direction);
        assert(comm_diag.fw_deg == 34 || comm_diag.fw_deg == 35);
        guard_ticks = 3;
        comm_scheduler_schedule_sector_event(clock_tick, direction);
        assert(comm_diag.fw_deg == 0);
        guard_ticks = 0;
        comm_scheduler_schedule_sector_event(clock_tick, -direction);
        assert(comm_diag.fw_deg == 0);
    }

    // A compare already elapsed while software was calculating/programming it.
    reset(); clock_tick = 4; comm_arm(65535);
    assert((sr & TIM_SR_CC2IF) && software_events == 1);
    // Clear the previous event before arming a future edge.
    comm_arm(10); assert(!(sr & TIM_SR_CC2IF));
    assert(ccr == 10 && (dier & TIM_DIER_CC2IE));
    // A large phase step is applied at once, in both directions.
    for (int direction = -1; direction <= 1; direction += 2) {
        reset(); set_observer(0, 50000, direction);
        comm_scheduler_schedule_sector_event(clock_tick, direction);
        observer_state.pos += (uint32_t)(direction * 3000) << 16;
        comm_scheduler_schedule_sector_event(clock_tick, direction);
        assert(hall == expected_sector(direction));
        // The ISR may fire while the new phase is being calculated.
        for (unsigned count = 1; count <= 6; count++) {
            reset(); set_observer(0, 50000, direction);
            comm_scheduler_schedule_sector_event(clock_tick, direction);
            observer_state.pos += (uint32_t)(direction * 600) << 16;
            preempt_on_mask = count;
            comm_scheduler_schedule_sector_event(clock_tick, direction);
            assert(!stopped && hall == expected_sector(direction));
        }
    }
    // A delayed interrupt selects the CURRENT sector, not a replayed next one.
    reset(); set_observer(0, 50000, 1);
    comm_scheduler_schedule_sector_event(clock_tick, 1);
    uint16_t late = (uint16_t)(comm_scheduler.next_comm_tick + 80);
    clock_tick = late;
    sr |= TIM_SR_CC2IF; tim21_isr();
    assert(!stopped && hall == expected_sector(1));
    // Loss of measurements stops even when compare timing is otherwise healthy.
    reset(); set_observer(0, 50000, 1);
    comm_scheduler_schedule_sector_event(clock_tick, 1);
    clock_tick = comm_scheduler.next_comm_tick;
    comm_scheduler.position_tick = clock_tick - SENSOR_STALE_US - 1;
    sr |= TIM_SR_CC2IF; tim21_isr();
    assert(stopped == 1 && captures_finished == 1);
    // An impossible speed estimate faults instead of pulsing PWM each tick.
    reset(); observer_state.vel = INT32_MAX; observer_state.init = 2;
    comm_scheduler_schedule_sector_event(clock_tick, 1);
    assert(stopped == 1 && current_duty == 0);
    return 0;
}
