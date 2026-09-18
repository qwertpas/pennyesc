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
static int current_mode = PNY_MODE_RUN;
static int current_lead_turn16, duty;
static bool braked = true, irq = true;
static uint8_t ready_result;
static uint8_t run_ready(void) { return ready_result; }
static void systick_interrupt_disable(void) { irq = false; }
static void systick_interrupt_enable(void) { irq = true; }
static int lead_deg_to_turn16(int value) { return value; }
static void control_set_duty(int value, int clip) { assert(clip == DUTY_LIMIT); duty = value; }
static void release_brake(void) { assert(!irq); braked = false; }
static void run_begin(void) { current_mode = PNY_MODE_RUN; }
static void control_disable(void) { duty = 0; }
static int32_t velocity_turn32_per_s;
static uint16_t current_angle_turn16;
static int16_t turn32_per_s_to_rpm(int32_t value) { return value; }
/* FUNCTIONS */
int main(void) {
    pny_capture_start_payload_t request = {PNY_DEBUG_CAPTURE_START, 400, 90, 200, 1000};
    ready_result = PNY_RESULT_BAD_STATE;
    assert(capture_start((void *)&request, sizeof(request)) == PNY_RESULT_BAD_STATE);
    assert(braked && !capture_state.active);
    ready_result = PNY_RESULT_OK;
    assert(capture_start((void *)&request, sizeof(request)) == PNY_RESULT_OK);
    assert(!braked && irq && duty == 400 && capture_state.active);
    for (unsigned i = 0; i < 200; i++) {
        current_angle_turn16 = i * 100;
        velocity_turn32_per_s = i * 120;
        capture_tick_1ms();
        assert(capture_state.samples[i].angle_turn16 == i * 100);
        assert(capture_state.samples[i].rpm == i * 120);
        assert(duty == (i == 199 ? 0 : 400));
    }
    assert(!capture_state.active && capture_state.done);
    assert(capture_state.sample_count == 200 && capture_state.missed_count == 0);
    capture_tick_1ms();
    assert(capture_state.sample_count == 200);
    return 0;
}
