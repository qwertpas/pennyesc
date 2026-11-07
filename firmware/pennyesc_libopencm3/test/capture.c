#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pennyesc_protocol.h"

#define CAPTURE_MAX_DURATION_MS 2000
#define ADVANCE_MIN_DEG -180
#define ADVANCE_MAX_DEG 180
#define DUTY_LIMIT 799
/* STATE */
static capture_state_t capture_state;
static int current_mode = PNY_MODE_IDLE;
static int current_lead_turn16, duty;
static bool braked = true, irq = true;
static uint8_t ready_result;
static uint32_t system_millis;
static uint8_t run_ready(void) { return ready_result; }
#define irq_lock() (irq = false)
#define irq_unlock() (irq = true)
static int lead_deg_to_turn16(int value) { return value; }
static void control_set_duty(int value, int clip) { assert(clip == DUTY_LIMIT); duty = value; }
static void release_brake(void) { assert(!irq); braked = false; }
static void run_begin(void) { current_mode = PNY_MODE_RUN; }
static void control_disable(void) { duty = 0; }
/* FUNCTIONS */
static pny_capture_start_payload_t request(int16_t d, uint16_t ms, uint16_t decimation) {
    pny_capture_start_payload_t r = {PNY_DEBUG_CAPTURE_START, d, 90, ms, decimation};
    return r;
}
int main(void) {
    // A run with a duty owns the motor and stops it when the window ends.
    pny_capture_start_payload_t r = request(400, 5, 2);
    ready_result = PNY_RESULT_BAD_STATE;
    assert(capture_start((void *)&r, sizeof(r)) == PNY_RESULT_BAD_STATE);
    assert(braked && !capture_state.active);
    ready_result = PNY_RESULT_OK;
    assert(capture_start((void *)&r, sizeof(r)) == PNY_RESULT_OK);
    assert(!braked && irq && duty == 400 && capture_state.active && current_mode == PNY_MODE_RUN);
    for (unsigned i = 0; i < 20; i++) {
        capture_record((uint16_t)(i * 100), (int16_t)i);   // decimation 2 keeps odd ticks
    }
    assert(capture_state.sample_count == 10);
    assert(capture_state.samples[0].angle_turn16 == 100 && capture_state.samples[0].rpm == 1);
    system_millis += 5;
    capture_record(9999, 0);
    capture_record(9999, 0);
    assert(!capture_state.active && capture_state.done && duty == 0);

    // A duty-0 recorder rings until a trigger, then keeps a quarter buffer.
    r = request(0, 0, 1);
    duty = 123;
    assert(capture_start((void *)&r, sizeof(r)) == PNY_RESULT_OK);
    assert(capture_state.active && duty == 123 && capture_state.trigger_index == UINT16_MAX);
    for (unsigned i = 0; i < 1000; i++) {
        capture_record((uint16_t)i, (int16_t)i);
    }
    assert(capture_state.active && capture_state.sample_count == PNY_CAPTURE_MAX_SAMPLES);
    capture_trigger();
    capture_trigger();   // a second event does not move the trigger
    for (unsigned i = 1000; capture_state.active; i++) {
        capture_record((uint16_t)i, (int16_t)i);
    }
    assert(capture_state.done && duty == 123);   // a recorder never stops the motor
    assert(capture_state.trigger_index == PNY_CAPTURE_MAX_SAMPLES / 4u);
    // Oldest record sits at write_index: 1000 + 50 records written, last 200 kept.
    assert(capture_state.samples[capture_state.write_index].angle_turn16 == 850);

    // Duty-0 recorders need no duration; a duty run does.
    r = request(100, 0, 1);
    assert(capture_start((void *)&r, sizeof(r)) == PNY_RESULT_RANGE);
    r = request(0, CAPTURE_MAX_DURATION_MS + 1, 1);
    assert(capture_start((void *)&r, sizeof(r)) == PNY_RESULT_RANGE);
    r = request(0, 10, 256);
    assert(capture_start((void *)&r, sizeof(r)) == PNY_RESULT_RANGE);
    return 0;
}
