#include <Arduino.h>
#include <Wire.h>
#include "pennyesc_arduino_debug.h"

static const int ESC_RX_PIN = 13;
static const int ESC_TX_PIN = 12;
static const int INA_GND_PIN = 7;
static const int INA_PWR_PIN = 6;
static const int INA_SDA_PIN = 5;
static const int INA_SCL_PIN = 4;
static const uint8_t ESC_ADDR = 3;
static const uint8_t INA_ADDR = 0x40;
static const float SHUNT_OHMS = 0.1f;
static const float CURRENT_LSB_A = 0.0001f;
static const int16_t CLIP = 500;
static const int16_t BASELINE_ADVANCE_DEG = 90;
static const int16_t OBSERVER_LEAD_US = 180;
static const uint16_t PRESPIN_MS = 60;
static const uint16_t RUN_MS = 200;
static const uint32_t TRANSITION_US = 60000u;
static const uint16_t REST_MS = 1000;
static const uint16_t SAMPLE_US = 1000;
static const uint16_t MAX_SAMPLES = PNY_CAPTURE_MAX_SAMPLES;

struct InaData {
    float bus_v = 0.0f;
    float current_a = 0.0f;
    float power_w = 0.0f;
    bool ok = false;
};

struct SweepSample {
    uint32_t t_us = 0;
    uint32_t dt_us = 0;
    float rpm = 0.0f;
    float bus_v = 0.0f;
    float current_a = 0.0f;
    float power_w = 0.0f;
    bool rpm_ok = false;
    bool ina_ok = false;
};

static PennyEscDebug esc(ESC_ADDR);
static String input_line;
static SweepSample samples[MAX_SAMPLES];

static bool writeReg(uint8_t reg, uint16_t value)
{
    Wire.beginTransmission(INA_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)value);
    return Wire.endTransmission() == 0;
}

static bool readReg(uint8_t reg, uint16_t &value)
{
    Wire.beginTransmission(INA_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }
    if (Wire.requestFrom((int)INA_ADDR, 2) != 2) {
        return false;
    }
    value = (uint16_t)Wire.read() << 8;
    value |= (uint16_t)Wire.read();
    return true;
}

static bool initIna()
{
    uint16_t cal = (uint16_t)(0.00512f / (CURRENT_LSB_A * SHUNT_OHMS));
    return writeReg(0x00, 0x4127) && writeReg(0x05, cal);
}

static InaData readIna()
{
    uint16_t raw_bus = 0;
    uint16_t raw_current = 0;
    InaData data;

    data.ok = readReg(0x02, raw_bus) && readReg(0x04, raw_current);
    if (!data.ok) {
        return data;
    }

    data.bus_v = (float)raw_bus * 0.00125f;
    data.current_a = (float)(int16_t)raw_current * CURRENT_LSB_A;
    data.power_w = data.bus_v * data.current_a;
    return data;
}

static bool setDuty(int16_t duty)
{
    PennyEscStatus status;
    return esc.setControl(0.0f, 0.0f, 0.0f, duty, CLIP, &status, 150u);
}

static void stopMotor()
{
    PennyEscStatus status;
    esc.setControl(0.0f, 0.0f, 0.0f, 0, CLIP, &status, 150u);
}

static void printHeader()
{
    Serial.println(
        "run_id,duty_cmd,observer_lead_us,baseline_advance_deg,test_advance_deg,transition_us,advance_start_us,advance_done_us,advance_set_ok,capture_ok,sample,t_us,dt_us,command_advance_deg,rpm_ok,ina_ok,rpm,bus_v,current_a,power_w"
    );
}

static SweepSample readInaSample(uint32_t start_us, uint32_t prev_us)
{
    uint32_t t_us = micros();
    InaData ina = readIna();
    SweepSample sample;

    sample.t_us = t_us - start_us;
    sample.dt_us = t_us - prev_us;
    sample.ina_ok = ina.ok;
    sample.bus_v = ina.bus_v;
    sample.current_a = ina.current_a;
    sample.power_w = ina.power_w;
    return sample;
}

static void printSample(
    uint16_t run_id,
    int16_t duty,
    int16_t advance,
    uint32_t advance_start_us,
    uint32_t advance_done_us,
    bool advance_set_ok,
    bool capture_ok,
    uint16_t sample_index,
    const SweepSample &sample
)
{
    int16_t command_advance = sample_index < (TRANSITION_US / SAMPLE_US) ? BASELINE_ADVANCE_DEG : advance;

    Serial.printf(
        "%u,%d,%d,%d,%d,%u,%lu,%lu,%u,%u,%u,%lu,%lu,%d,%u,%u,%.2f,%.5f,%.5f,%.5f\n",
        run_id,
        duty,
        OBSERVER_LEAD_US,
        BASELINE_ADVANCE_DEG,
        advance,
        TRANSITION_US,
        (unsigned long)advance_start_us,
        (unsigned long)advance_done_us,
        advance_set_ok ? 1u : 0u,
        capture_ok ? 1u : 0u,
        sample_index,
        (unsigned long)sample.t_us,
        (unsigned long)sample.dt_us,
        command_advance,
        sample.rpm_ok ? 1u : 0u,
        sample.ina_ok ? 1u : 0u,
        sample.rpm,
        sample.bus_v,
        sample.current_a,
        sample.power_w
    );
}

static bool startRpmCapture(int16_t duty, int16_t advance, PennyEscCaptureStatus &status)
{
    return esc.startCapture(duty, advance, RUN_MS, 1000u, status, 200u) &&
        status.result == PNY_RESULT_OK;
}

static bool setObserver()
{
    PennyEscCaptureStatus status;

    return esc.setObserver(OBSERVER_LEAD_US, PNY_OBSERVER_AB_FAST, &status, 200u);
}

static bool refreshCapture(PennyEscCaptureStatus &status)
{
    return esc.getCaptureStatus(status, 200u);
}

static uint16_t readCaptureSamples(uint16_t count)
{
    uint16_t offset = 0;
    while (offset < count && offset < MAX_SAMPLES) {
        PennyEscCaptureSample capture_samples[PNY_CAPTURE_READ_MAX_SAMPLES];
        uint8_t got = 0u;
        uint8_t want = (uint8_t)(count - offset);
        if (want > 14u) {
            want = 14u;
        }
        if (!esc.readCapture(offset, capture_samples, want, got, 200u) || got == 0u) {
            break;
        }
        for (uint8_t i = 0; i < got && offset < MAX_SAMPLES; i++) {
            samples[offset].rpm = (float)capture_samples[i].rpm;
            samples[offset].rpm_ok = true;
            offset++;
        }
    }
    return offset;
}

static uint16_t runSegment(uint16_t run_id, int16_t duty, int16_t advance, bool emit_csv)
{
    for (uint16_t i = 0; i < MAX_SAMPLES; i++) {
        samples[i] = SweepSample();
    }

    PennyEscStatus status;
    bool prespin_advance_ok = esc.setAdvanceDeg(BASELINE_ADVANCE_DEG, &status, 150u);
    bool prespin_duty_ok = setDuty(duty);
    if (!prespin_advance_ok || !prespin_duty_ok) {
        Serial.printf("# prespin_failed run_id=%u duty=%d advance=%d advance_ok=%u duty_ok=%u\n",
            run_id,
            duty,
            BASELINE_ADVANCE_DEG,
            prespin_advance_ok ? 1u : 0u,
            prespin_duty_ok ? 1u : 0u
        );
    }
    delay(PRESPIN_MS);

    PennyEscCaptureStatus capture_status;
    bool capture_ok = startRpmCapture(duty, BASELINE_ADVANCE_DEG, capture_status);
    if (!capture_ok) {
        Serial.printf("# capture_start_failed run_id=%u duty=%d advance=%d result=%u\n",
            run_id,
            duty,
            BASELINE_ADVANCE_DEG,
            capture_status.result
        );
    }

    uint32_t start_us = micros();
    uint32_t next_us = start_us;
    uint32_t prev_us = start_us;
    uint16_t sample_count = 0;
    bool advance_set = false;
    bool advance_set_ok = false;
    uint32_t advance_start_us = 0;
    uint32_t advance_done_us = 0;

    while ((uint32_t)(micros() - start_us) < (uint32_t)RUN_MS * 1000u) {
        int32_t wait_us = (int32_t)(next_us - micros());
        if (wait_us > 0) {
            delayMicroseconds((uint32_t)wait_us);
        }
        if (!advance_set && (uint32_t)(micros() - start_us) >= TRANSITION_US) {
            PennyEscStatus status;
            advance_start_us = micros() - start_us;
            advance_set_ok = esc.setAdvanceDeg(advance, &status, 50u);
            advance_done_us = micros() - start_us;
            advance_set = true;
        }
        uint32_t sample_us = micros();
        if (sample_count < MAX_SAMPLES) {
            samples[sample_count] = readInaSample(start_us, prev_us);
        }
        prev_us = sample_us;
        sample_count++;
        next_us += SAMPLE_US;
    }

    uint32_t deadline_ms = millis() + 300u;
    while (capture_ok && (int32_t)(millis() - deadline_ms) < 0) {
        if (refreshCapture(capture_status) && !capture_status.active) {
            break;
        }
        delay(2);
    }
    stopMotor();

    uint16_t saved_samples = sample_count > MAX_SAMPLES ? MAX_SAMPLES : sample_count;
    if (capture_ok) {
        uint16_t rpm_count = readCaptureSamples(capture_status.sample_count);
        if (rpm_count > saved_samples) {
            saved_samples = rpm_count;
        }
    }

    if (emit_csv) {
        for (uint16_t sample_index = 0; sample_index < saved_samples; sample_index++) {
            printSample(
                run_id,
                duty,
                advance,
                advance_start_us,
                advance_done_us,
                advance_set_ok,
                capture_ok,
                sample_index,
                samples[sample_index]
            );
        }
    }

    delay(REST_MS);
    return saved_samples;
}

static void runRateCheck()
{
    stopMotor();
    uint32_t start_ms = millis();
    uint32_t loops = 0;
    uint32_t max_us = 0;
    uint32_t sum_us = 0;
    while ((uint32_t)(millis() - start_ms) < 1000u) {
        uint32_t t0 = micros();
        PennyEscEncoderData data;
        (void)esc.getPosVel(data, 5u);
        (void)readIna();
        uint32_t elapsed = micros() - t0;
        if (elapsed > max_us) {
            max_us = elapsed;
        }
        sum_us += elapsed;
        loops++;
    }
    Serial.printf("# rate_check loops=%lu avg_us=%lu max_us=%lu possible_1000hz=%u\n",
        (unsigned long)loops,
        loops ? (unsigned long)(sum_us / loops) : 0ul,
        (unsigned long)max_us,
        max_us <= SAMPLE_US ? 1u : 0u
    );
}

static void runSweep()
{
    uint16_t run_id = 0;
    Serial.println("# sweep_start");
    printHeader();
    for (int16_t abs_duty = 100; abs_duty <= 300; abs_duty += 20) {
        for (int8_t sign = -1; sign <= 1; sign += 2) {
            int16_t duty = (int16_t)(sign * abs_duty);
            for (int16_t advance = 0; advance <= 180; advance += 10) {
                runSegment(run_id, duty, advance, true);
                run_id++;
            }
        }
    }
    stopMotor();
    Serial.println("# sweep_done");
}

static void handleLine(String line)
{
    line.trim();
    if (line == "rate") {
        runRateCheck();
    } else if (line == "sweep") {
        runSweep();
    } else if (line.startsWith("one ")) {
        int split = line.indexOf(' ', 4);
        if (split < 0) {
            Serial.println("# usage: one <duty> <advance>");
            return;
        }
        int16_t duty = (int16_t)line.substring(4, split).toInt();
        int16_t advance = (int16_t)line.substring(split + 1).toInt();
        Serial.println("# one_start");
        printHeader();
        runSegment(0, duty, advance, true);
        stopMotor();
        Serial.println("# one_done");
    } else if (line == "stop") {
        stopMotor();
        Serial.println("# stopped");
    } else {
        Serial.println("# commands: rate sweep one <duty> <advance> stop");
    }
}

void setup()
{
    Serial.begin(921600);
    esc.begin(Serial1, ESC_RX_PIN, ESC_TX_PIN);

    pinMode(INA_GND_PIN, OUTPUT);
    digitalWrite(INA_GND_PIN, LOW);
    pinMode(INA_PWR_PIN, OUTPUT);
    digitalWrite(INA_PWR_PIN, HIGH);
    delay(20);

    Wire.begin(INA_SDA_PIN, INA_SCL_PIN);
    Wire.setClock(1000000);
    bool ina_ok = initIna();
    bool observer_ok = setObserver();

    delay(100);
    stopMotor();
    Serial.printf("# advance_transition_sweep esc=%u ina=%u observer=%u baseline_advance=%d observer_lead_us=%d baud=921600\n",
        ESC_ADDR,
        ina_ok ? 1u : 0u,
        observer_ok ? 1u : 0u,
        BASELINE_ADVANCE_DEG,
        OBSERVER_LEAD_US
    );
    Serial.println("# commands: rate sweep one <duty> <advance> stop");
}

void loop()
{
    while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        if (ch == '\n' || ch == '\r') {
            if (input_line.length()) {
                handleLine(input_line);
                input_line = "";
            }
        } else if (input_line.length() < 32) {
            input_line += ch;
        }
    }
}
