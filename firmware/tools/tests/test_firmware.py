"""Run production C timing/parser code on a mocked timer, with UB checks."""
import json
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

FIRMWARE = Path(__file__).resolve().parents[2]
STM32 = FIRMWARE / "pennyesc_libopencm3"


def function(source, name):
    match = re.search(r"^(?:static )?[^\n;{}]*\b" + name + r"\([^;]*?\)\n\{", source, re.M)
    if not match:
        raise ValueError(name)
    end = match.end()
    depth = 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end] + "\n"


class FirmwareTests(unittest.TestCase):
    def run_c(self, source):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "check.c"
            path.write_text(source)
            binary = Path(folder) / "check"
            subprocess.run(["cc", "-std=c11", "-O1", "-g", "-fsanitize=undefined",
                            "-fno-sanitize-recover=all", "-I", str(STM32 / "include"),
                            "-I", str(FIRMWARE / "Lib"), str(path), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)

    def test_calibration_after_brake(self):
        main = (STM32 / "src/main.c").read_text()
        source = '''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pennyesc_protocol.h"
#define PNY_DRIVE_BRUSHED 0
static uint8_t current_mode;
static bool scheduler, brake, sensor_ok = true;
static struct { bool active; uint8_t total_points, sweep_dir, index; } cal_state;
static void control_disable(void) {}
static void run_stop(void) { scheduler = false; current_mode = PNY_MODE_IDLE; }
static bool sensor_init_full_mode(void) { assert(!scheduler); return sensor_ok; }
static void mct_apply_config(void) {}
static void delay_ms(unsigned ms) { (void)ms; }
static void release_brake(void) { brake = false; }
static void calibration_arm_point(uint8_t index) { cal_state.index = index; }
''' + function(main, "calibration_start") + '''
int main(void) {
    current_mode = PNY_MODE_RUN; scheduler = true; brake = true;
    assert(calibration_start(0) == PNY_RESULT_OK);
    assert(current_mode == PNY_MODE_CAL && cal_state.active);
    assert(!scheduler && !brake && cal_state.index == 0);
    assert(calibration_start(1) == PNY_RESULT_BAD_STATE);
    current_mode = PNY_MODE_IDLE;
    assert(calibration_start(2) == PNY_RESULT_BAD_ARG);
    sensor_ok = false;
    assert(calibration_start(0) == PNY_RESULT_BAD_STATE);
    assert(current_mode == PNY_MODE_IDLE && !cal_state.active);
    sensor_ok = true;
    assert(calibration_start(1) == PNY_RESULT_OK && cal_state.sweep_dir == 1);
    return 0;
}
'''
        self.run_c(source)

    def test_capture(self):
        main = (STM32 / "src/main.c").read_text()
        state = main[main.index("typedef struct {\n    volatile bool active;\n    volatile bool done;"):]
        state = state[:state.index("} capture_state_t;") + len("} capture_state_t;")]
        source = (STM32 / "test/capture.c").read_text()
        self.run_c(source.replace("/* STATE */", state).replace(
            "/* FUNCTIONS */", "\n".join(function(main, name) for name in
                                         ["capture_start", "capture_tick_1ms"])))

    def test_timing(self):
        main = (STM32 / "src/main.c").read_text()
        names = ["comm_fault", "position_target_direction", "abs_u32", "observer_ab_update", "observer_position_at_tick", "commutation_phase_at_tick", "comm_sector_start_phase",
                 "comm_arm", "comm_schedule_absolute", "comm_scheduler_schedule_sector_event", "comm_control_tick", "tim21_isr"]
        functions = "\n".join(function(main, name) for name in names)
        # Use the actual shared ISR state layout as well as the actual functions.
        state = main[main.index("typedef struct {\n    volatile bool active;\n    volatile uint8_t sector;"):]
        state = state[:state.index("} comm_scheduler_t;") + len("} comm_scheduler_t;")]
        source = (STM32 / "test/timing.c").read_text()
        self.run_c(source.replace("/* STATE */", state).replace("/* FUNCTIONS */", functions))

    def test_l011_timers(self):
        # ST's device-specific CMSIS header lists TIM2 and TIM21, not TIM22:
        # https://github.com/STMicroelectronics/cmsis-device-l0/blob/master/Include/stm32l011xx.h
        board = json.loads((STM32 / "boards/custom_l011.json").read_text())
        self.assertEqual(board["build"]["mcu"], "stm32l011f4u6")
        for name in ["main.c", "tmag5273.c", "pennyesc_boot.c"]:
            source = (STM32 / "src" / name).read_text()
            source = re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.S)
            timers = set(re.findall(r"\b(?:RCC_|NVIC_)?(TIM\d+)(?:_IRQ)?\b", source))
            self.assertFalse(timers - {"TIM2", "TIM21"}, (name, timers))

    def test_startup_control_tick(self):
        main = (STM32 / "src/main.c").read_text()
        state = main[main.index("typedef struct {\n    volatile bool active;\n    volatile uint8_t sector;"):]
        state = state[:state.index("} comm_scheduler_t;") + len("} comm_scheduler_t;")]
        names = ["systick_setup", "comm_scheduler_start", "comm_scheduler_stop", "sys_tick_handler"]
        source = (STM32 / "test/startup.c").read_text()
        for tick in (100, 200):
            self.run_c(source.replace("#define SENSOR_TICK_US 100u", f"#define SENSOR_TICK_US {tick}u").replace(
                "/* STATE */", state).replace(
                "/* FUNCTIONS */", "\n".join(function(main, name) for name in names)))

    def test_sensor_initialization(self):
        driver = (STM32 / "src/tmag5273.c").read_text()
        definitions = "\n".join(re.findall(
            r"^#define (?:REG_DEVICE_CONFIG_1|REG_DEVICE_STATUS|REG_DEVICE_ID|DEVICE_STATUS_VCC_UV|I2C_RD_STANDARD)\s+[^\n]+",
            driver, re.M))
        source = (STM32 / "test/sensor_init.c").read_text()
        self.run_c(source.replace("/* DEFINITIONS */", definitions).replace(
            "/* FUNCTIONS */", function(driver, "tmag5273_init")))

    def test_sensor_modes(self):
        driver = (STM32 / "src/tmag5273.c").read_text()
        definitions = "\n".join(re.findall(
            r"^#define (?:CONV_AVG_\w+|I2C_RD_\w+|MAG_CH_\w+|OP_\w+)\s+[^\n]+",
            driver, re.M))
        source = '''
#include <assert.h>
#include "tmag5273.h"
static unsigned config;
static bool write_mode_regs(uint8_t avg, uint8_t read, uint8_t channels, uint8_t mode) {
    config = (avg << 12) | (read << 8) | (channels << 4) | mode;
    return true;
}
''' + definitions + '\n' + function(driver, "tmag5273_set_mode") + '''
int main(void) {
    assert(tmag5273_set_mode(TMAG5273_MODE_FAST_XY) && config == 0x0232);
    assert(tmag5273_set_mode(TMAG5273_MODE_FULL_XYZ) && config == 0x1072);
#if PNY_DRIVE_BRUSHED
    assert(tmag5273_set_mode(TMAG5273_MODE_FAST_XYZ) && config == 0x0172);
#endif
    return 0;
}
'''
        for brushed in (0, 1):
            self.run_c(f"#define PNY_DRIVE_BRUSHED {brushed}\n" + source)

    def test_sensor(self):
        driver = (STM32 / "src/tmag5273.c").read_text()
        state = driver[driver.index("#ifndef PNY_DRIVE_BRUSHED"):driver.index("static void i2c1_recover(void);")]
        names = ["async_disable", "async_clear_flags", "i2c_start_transfer", "async_fail",
                 "tmag5273_async_cancel", "tmag5273_async_start_xy", "tmag5273_async_take_xy",
                 "tmag5273_i2c1_isr"]
        source = (STM32 / "test/sensor.c").read_text()
        self.run_c(source.replace("/* STATE */", state).replace(
            "/* FUNCTIONS */", "\n".join(function(driver, name) for name in names)))

    def test_reply_buffer_reuse(self):
        main = (STM32 / "src/main.c").read_text()
        state = main[main.index("static union {"):main.index("static uint32_t uart_quiet_until_ms;")]
        functions = "\n".join(function(main, name) for name in
                              ["send_status_response", "handle_capture_read", "handle_cal_read_blob"])
        self.run_c('''
#include <assert.h>
#include <string.h>
#include "pennyesc_frame.h"
#include "pennyesc_calibration.h"
static pennyesc_calibration_blob_t calibration;
static bool calibrated = true;
const pennyesc_calibration_blob_t *pennyesc_calibration_active(void) {
    return calibrated ? &calibration : NULL;
}
static struct {
    bool active;
    uint16_t sample_count;
    pny_capture_sample_t samples[3];
} capture_state;
static uint8_t sent[64], sent_len;
static void send_frame(uint8_t cmd, const void *data, uint8_t len) {
    (void)cmd; memcpy(sent, data, len); sent_len = len;
}
static void fill_status_payload(pny_status_payload_t *out, uint8_t result) {
    memset(out, 0x55, sizeof(*out)); out->result = result;
}
''' + state + functions + '''
int main(void) {
    frame_parser.last_byte_ms = 123;
    capture_state.sample_count = 3;
    capture_state.samples[1].angle_turn16 = 12345;
    capture_state.samples[1].rpm = -1234;
    uint8_t *request = frame_parser.buf + 3;
    request[0] = PNY_DEBUG_CAPTURE_READ;
    request[1] = 1; request[2] = 0; request[3] = 1;
    handle_capture_read(request, 4);
    assert(sent_len == 9 && sent[0] == PNY_DEBUG_CAPTURE_READ && sent[1] == PNY_RESULT_OK);
    pny_capture_read_payload_t reply = {0}; memcpy(&reply, sent, sent_len);
    assert(reply.offset == 1 && reply.count == 1);
    assert(reply.samples[0].angle_turn16 == 12345 && reply.samples[0].rpm == -1234);
    handle_capture_read(NULL, 0);
    assert(sent_len == 5 && sent[1] == PNY_RESULT_BAD_ARG);
    send_status_response(PNY_CMD_GET_STATUS, PNY_RESULT_OK);
    assert(sent_len == 60 && sent[0] == PNY_RESULT_OK);
    assert(frame_parser.idx == 0 && frame_parser.expected == 0 && frame_parser.last_byte_ms == 123);
    for (unsigned i = 0; i < sizeof(calibration); i++) ((uint8_t *)&calibration)[i] = (uint8_t)i;
    request[0] = PNY_CAL_READ_BLOB;
    request[1] = 0x50; request[2] = 2; request[3] = 48;
    handle_cal_read_blob(request, 4);
    assert(sent_len == 49 && sent[0] == PNY_RESULT_OK);
    assert(memcmp(sent + 1, (uint8_t *)&calibration + 592, 48) == 0);
    request[1] = 0x80; request[2] = 2; request[3] = 1;
    handle_cal_read_blob(request, 4);
    assert(sent_len == 1 && sent[0] == PNY_RESULT_BAD_ARG);
    request[1] = 0; request[2] = 0; request[3] = 64;
    handle_cal_read_blob(request, 4);
    assert(sent_len == 1 && sent[0] == PNY_RESULT_BAD_ARG);
    request[3] = 63;
    handle_cal_read_blob(request, 4);
    assert(sent_len == 64 && sent[0] == PNY_RESULT_OK);
    calibrated = false;
    request[1] = 0; request[2] = 0; request[3] = 1;
    handle_cal_read_blob(request, 4);
    assert(sent_len == 1 && sent[0] == PNY_RESULT_NOT_CALIBRATED);
    handle_cal_read_blob(NULL, 0);
    assert(sent_len == 1 && sent[0] == PNY_RESULT_BAD_ARG);
    return 0;
}
''')

    def test_frame_parser(self):
        source = (STM32 / "src/pennyesc_frame.c").read_text()
        functions = "\n".join(function(source, name) for name in
                              ["crc8_byte", "pny_frame_crc8", "pny_frame_parser_reset", "pny_frame_parser_push", "pny_frame_send"])
        self.run_c('''
#include <assert.h>
#include <string.h>
#include <stddef.h>
#include "pennyesc_frame.h"
_Static_assert(sizeof(pny_status_payload_t) == 60, "status wire size");
_Static_assert(offsetof(pny_status_payload_t, isr_overrun_count) == 34, "status wire offset");
#define USART2 0
static uint8_t tx[68];
static unsigned tx_count;
static void uart_tx_start(void) { tx_count = 0; }
static void uart_tx_stop(void) {}
static void usart_send_blocking(int uart, uint8_t byte) { (void)uart; tx[tx_count++] = byte; }
''' + functions + '''
int main(void) {
    pny_frame_parser_t parser = {0};
    const uint8_t *frame;
    uint8_t len;
    for (unsigned header = 0; header < 256; ++header) {
        uint8_t input[] = {0xaa, header, 2, 0xaa, 0x55, 0};
        input[5] = pny_frame_crc8(input, 5);
        for (unsigned i = 0; i < sizeof(input); ++i) {
            bool ready = pny_frame_parser_push(&parser, input[i], i, 10, &frame, &len);
            assert(ready == (i == 5));
        }
        assert(len == sizeof(input) && memcmp(frame, input, len) == 0);
    }
    uint8_t bad[] = {0xaa, 0x11, 0, 0};
    for (unsigned i = 0; i < sizeof(bad); ++i)
        assert(!pny_frame_parser_push(&parser, bad[i], i, 10, &frame, &len));
    assert(parser.crc_error);
    pny_frame_parser_push(&parser, 0xaa, 0, 10, &frame, &len);
    pny_frame_parser_push(&parser, 0x11, 11, 10, &frame, &len);
    assert(parser.idx == 0);
    uint8_t payload[64];
    for (unsigned i = 0; i < sizeof(payload); ++i) payload[i] = i;
    for (unsigned size = 0; size <= sizeof(payload); ++size) {
        pny_frame_send(0xaa, payload, size);
        assert(tx_count == size + 4 && tx[0] == 0xaa && tx[1] == 0xaa && tx[2] == size);
        assert(memcmp(tx + 3, payload, size) == 0);
        assert(tx[tx_count - 1] == pny_frame_crc8(tx, tx_count - 1));
    }
    return 0;
}
''')
