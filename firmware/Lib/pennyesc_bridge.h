#ifndef PENNYESC_BRIDGE_H
#define PENNYESC_BRIDGE_H

// Stateless USB-to-UART bridge for PennyESC.
//
// Every message is a packet: [type, payload..., crc8]. USB packets are COBS
// encoded and surrounded by 0x00 delimiters; BLE packets are one GATT write or
// notification each. The bridge keeps no protocol mode: each UART packet names
// the baud it must be sent at, so a bridge reset or a host reload never leaves
// the two sides disagreeing.

#include <Arduino.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include "pennyesc_arduino.h"

static const uint8_t PENNY_BRIDGE_VERSION = 2u;

enum : uint8_t {
    PENNY_PKT_HELLO = 0x01,      // -> STATE
    PENNY_PKT_PINS = 0x02,       // int8 rx, int8 tx, int8 gnd -> STATE
    PENNY_PKT_AUX = 0x03,        // int8 pin, uint16 pulse_us (0 = off) -> STATE
    PENNY_PKT_STREAM = 0x04,     // uint8 address, uint8 hz (0 = off) -> STATE
    PENNY_PKT_UART = 0x10,       // uint8 baud, bytes to write; no reply
    PENNY_PKT_UART_RX = 0x11,    // bridge -> host: bytes received on the UART
    PENNY_PKT_TELEMETRY = 0x12,  // bridge -> host: repeated [uint32 stamp_us, ESC status frame]
    PENNY_PKT_STATE = 0x81,      // bridge -> host: reply to every command
};

enum : uint8_t { PENNY_BAUD_APP = 0u, PENNY_BAUD_BOOT = 1u };

enum : uint8_t {
    PENNY_BRIDGE_OK = 0u,
    PENNY_BRIDGE_INVALID = 1u,
    PENNY_BRIDGE_SAVE_FAILED = 2u,
    PENNY_BRIDGE_PWM_FAILED = 3u,
    PENNY_BRIDGE_UNKNOWN = 4u,
    PENNY_BRIDGE_BUSY = 5u,
};

class PennyBridge {
public:
    // A host connection: USB, BLE, or anything else that carries packets.
    class Link {
    public:
        virtual ~Link() {}
        virtual void send(const uint8_t *packet, size_t length) = 0;
        virtual size_t maxPacket() const = 0;
    };

    virtual ~PennyBridge() {}

    void begin(Stream &usb, HardwareSerial &uart = Serial1, int rx = 12, int tx = 13, int gnd = -1)
    {
        usb_.stream = &usb;
        uart_ = &uart;
        rx_ = rx;
        tx_ = tx;
        gnd_ = gnd;
        Preferences settings;
        if (settings.begin("pennyesc", true)) {
            uint32_t pins = settings.getUInt("pins", packPins(rx, tx, gnd));
            settings.end();
            int saved_rx = (int)(pins & 0xffu) - 1;
            int saved_tx = (int)((pins >> 8) & 0xffu) - 1;
            int saved_gnd = (int)((pins >> 16) & 0xffu) - 1;
            if (validPins(saved_rx, saved_tx, saved_gnd)) {
                rx_ = saved_rx;
                tx_ = saved_tx;
                gnd_ = saved_gnd;
            }
        }
        driveGround();
        setBaud(PENNY_BAUD_APP, true);
    }

    // Run poll() from a FreeRTOS task so a sketch can keep its own loop.
    void beginBackground(Stream &usb, HardwareSerial &uart = Serial1, int rx = 12, int tx = 13, int gnd = -1)
    {
        begin(usb, uart, rx, tx, gnd);
        xTaskCreatePinnedToCore(taskMain, "pnybridge", 4096, this, 1, 0, 0);
    }

    virtual void poll()
    {
        pollUsb();
        pollStream();
        pollUart();
        pennyesc_bridge_active = last_uart_ms_ != 0u && millis() - last_uart_ms_ < 1000u;
    }

    static uint8_t crc8(const uint8_t *data, size_t length) { return PennyEsc::crc8(data, length); }

    static size_t cobsEncode(const uint8_t *in, size_t length, uint8_t *out)
    {
        size_t code_index = 0u, out_index = 1u;
        uint8_t code = 1u;
        for (size_t i = 0u; i < length; i++) {
            if (in[i] == 0u) {
                out[code_index] = code;
                code_index = out_index++;
                code = 1u;
            } else {
                out[out_index++] = in[i];
                if (++code == 0xffu) {
                    out[code_index] = code;
                    code_index = out_index++;
                    code = 1u;
                }
            }
        }
        out[code_index] = code;
        return out_index;
    }

    // Returns the decoded length, or -1 for malformed input.
    static int cobsDecode(const uint8_t *in, size_t length, uint8_t *out)
    {
        size_t i = 0u, out_index = 0u;
        while (i < length) {
            uint8_t code = in[i++];
            if (code == 0u) return -1;
            for (uint8_t j = 1u; j < code; j++) {
                if (i >= length) return -1;
                out[out_index++] = in[i++];
            }
            if (code != 0xffu && i < length) out[out_index++] = 0u;
        }
        return (int)out_index;
    }

protected:
    static const size_t MAX_PACKET = 260u;

    class UsbLink : public Link {
    public:
        Stream *stream = &Serial;
        void send(const uint8_t *packet, size_t length) override
        {
            uint8_t encoded[MAX_PACKET + MAX_PACKET / 254u + 3u];
            encoded[0] = 0u;
            size_t count = cobsEncode(packet, length, encoded + 1u) + 1u;
            encoded[count++] = 0u;
            stream->write(encoded, count);
        }
        size_t maxPacket() const override { return 250u; }
    };

    static void taskMain(void *arg)
    {
        PennyBridge *bridge = (PennyBridge *)arg;
        for (;;) {
            bridge->poll();
            delay(1);
        }
    }

    // Packets from a host. Subclasses add other links and ownership rules.
    virtual bool accept(Link &link)
    {
        if (&link != owner_) {
            stopStream();
            owner_ = &link;
        }
        return true;
    }

    void handlePacket(Link &link, const uint8_t *packet, size_t length)
    {
        if (length < 2u || crc8(packet, length - 1u) != packet[length - 1u]) return;
        if (!accept(link)) {
            replyState(link, PENNY_BRIDGE_BUSY);
            return;
        }
        const uint8_t type = packet[0];
        const uint8_t *data = packet + 1;
        const size_t count = length - 2u;
        switch (type) {
        case PENNY_PKT_HELLO:
            stopStream();
            replyState(link, PENNY_BRIDGE_OK);
            break;
        case PENNY_PKT_PINS:
            replyState(link, count == 3u ? changePins((int8_t)data[0], (int8_t)data[1], (int8_t)data[2])
                                          : PENNY_BRIDGE_INVALID);
            break;
        case PENNY_PKT_AUX:
            replyState(link, count == 3u ? setAux((int8_t)data[0], data[1] | (data[2] << 8))
                                          : PENNY_BRIDGE_INVALID);
            break;
        case PENNY_PKT_STREAM:
            replyState(link, count == 2u ? startStream(data[0], data[1]) : PENNY_BRIDGE_INVALID);
            break;
        case PENNY_PKT_UART:
            if (count >= 1u && data[0] <= PENNY_BAUD_BOOT) {
                setBaud(data[0], false);
                if (count > 1u) uart().write(data + 1, count - 1u);
                if (count >= 3u && data[1] == PNY_FRAME_START) addresses_ |= (uint16_t)(1u << (data[2] >> 4));
                last_uart_ms_ = millis();
                if (last_uart_ms_ == 0u) last_uart_ms_ = 1u;
            }
            break;
        default:
            replyState(link, PENNY_BRIDGE_UNKNOWN);
            break;
        }
    }

    void sendPacket(Link &link, uint8_t type, const uint8_t *data, size_t length)
    {
        uint8_t packet[MAX_PACKET];
        if (length + 2u > sizeof(packet) || length + 2u > link.maxPacket()) return;
        packet[0] = type;
        if (length) memcpy(packet + 1, data, length);
        packet[length + 1u] = crc8(packet, length + 1u);
        link.send(packet, length + 2u);
    }

    void replyState(Link &link, uint8_t status)
    {
        uint8_t data[40];
        const char *board = boardName();
        size_t name = strlen(board);
        if (name > sizeof(data) - 9u) name = sizeof(data) - 9u;
        data[0] = status;
        data[1] = PENNY_BRIDGE_VERSION;
        data[2] = (uint8_t)(int8_t)rx_;
        data[3] = (uint8_t)(int8_t)tx_;
        data[4] = (uint8_t)(int8_t)gnd_;
        data[5] = (uint8_t)(int8_t)aux_pin_;
        data[6] = (uint8_t)(aux_pulse_ & 0xffu);
        data[7] = (uint8_t)(aux_pulse_ >> 8);
        data[8] = aux_enabled_ ? 1u : 0u;
        memcpy(data + 9, board, name);
        sendPacket(link, PENNY_PKT_STATE, data, 9u + name);
    }

    void pollUsb()
    {
        while (usb_.stream->available() > 0) {
            int value = usb_.stream->read();
            if (value < 0) break;
            if (value != 0) {
                if (usb_length_ < sizeof(usb_buf_)) usb_buf_[usb_length_] = (uint8_t)value;
                usb_length_++;
                continue;
            }
            if (usb_length_ != 0u && usb_length_ <= sizeof(usb_buf_)) {
                uint8_t packet[sizeof(usb_buf_)];
                int length = cobsDecode(usb_buf_, usb_length_, packet);
                if (length > 0) {
                    onUsbPacket();
                    handlePacket(usb_, packet, (size_t)length);
                }
            }
            usb_length_ = 0u;
        }
    }

    virtual void onUsbPacket() {}

    // ESC replies go to the host that last talked to the bridge. While the
    // bridge streams telemetry it parses frames so it can route its own polls.
    void pollUart()
    {
        uint8_t bytes[240];
        size_t limit = owner_ ? owner_->maxPacket() - 3u : sizeof(bytes);
        if (limit > sizeof(bytes)) limit = sizeof(bytes);
        size_t count = 0u;
        while (count < limit && uart().available() > 0) {
            bytes[count++] = (uint8_t)uart().read();
        }
        if (count != 0u && owner_ != 0) {
            if (stream_period_us_ == 0u) {
                uint8_t data[sizeof(bytes) + 1u];
                data[0] = baud_;
                memcpy(data + 1, bytes, count);
                sendPacket(*owner_, PENNY_PKT_UART_RX, data, count + 1u);
            } else {
                for (size_t i = 0u; i < count; i++) parseStreamByte(bytes[i]);
            }
        }
        if (batch_length_ != 0u && millis() - batch_ms_ >= 10u) flushTelemetry();
    }

    void parseStreamByte(uint8_t byte)
    {
        if (frame_length_ == 0u && byte != PNY_FRAME_START) return;
        frame_[frame_length_++] = byte;
        if (frame_length_ == 3u && frame_[2] > PNY_FRAME_MAX_PAYLOAD) {
            frame_length_ = 0u;
            return;
        }
        if (frame_length_ < 3u || frame_length_ != (size_t)frame_[2] + 4u) return;
        size_t length = frame_length_;
        frame_length_ = 0u;
        if (crc8(frame_, length - 1u) != frame_[length - 1u]) return;
        // Only the reply to the bridge's own poll is telemetry; a host's status request is forwarded.
        if (stream_waiting_ && micros() - stream_stamp_us_ < 5000u
            && (frame_[1] >> 4) == stream_address_ && (frame_[1] & 0x0fu) == PNY_CMD_GET_STATUS) {
            stream_waiting_ = false;
            addTelemetry(frame_, length);
        } else if (owner_ != 0) {
            uint8_t data[PNY_FRAME_MAX_PAYLOAD + 5u];
            data[0] = baud_;
            memcpy(data + 1, frame_, length);
            sendPacket(*owner_, PENNY_PKT_UART_RX, data, length + 1u);
        }
    }

    void addTelemetry(const uint8_t *frame, size_t length)
    {
        if (owner_ == 0) return;
        size_t limit = owner_->maxPacket() - 2u;
        if (limit > sizeof(batch_)) limit = sizeof(batch_);
        if (batch_length_ + 4u + length > limit) flushTelemetry();
        if (batch_length_ + 4u + length > limit) return;
        if (batch_length_ == 0u) batch_ms_ = millis();
        memcpy(batch_ + batch_length_, &stream_stamp_us_, 4u);
        memcpy(batch_ + batch_length_ + 4u, frame, length);
        batch_length_ += 4u + length;
    }

    void flushTelemetry()
    {
        if (batch_length_ != 0u && owner_ != 0) sendPacket(*owner_, PENNY_PKT_TELEMETRY, batch_, batch_length_);
        batch_length_ = 0u;
    }

    virtual uint8_t startStream(uint8_t address, uint8_t hz)
    {
        if (address > 15u || hz > 200u) return PENNY_BRIDGE_INVALID;
        stopStream();
        if (hz == 0u) return PENNY_BRIDGE_OK;
        setBaud(PENNY_BAUD_APP, false);
        stream_address_ = address;
        stream_period_us_ = 1000000u / hz;
        next_stream_us_ = micros();
        return PENNY_BRIDGE_OK;
    }

    void stopStream()
    {
        stream_period_us_ = 0u;
        stream_waiting_ = false;
        batch_length_ = 0u;
        frame_length_ = 0u;
    }

    void pollStream()
    {
        if (stream_period_us_ == 0u || (int32_t)(micros() - next_stream_us_) < 0) return;
        uint32_t now = micros();
        next_stream_us_ += stream_period_us_;
        if ((int32_t)(now - next_stream_us_) >= 0) next_stream_us_ = now + stream_period_us_;
        uint8_t frame[4] = {PNY_FRAME_START, (uint8_t)((stream_address_ << 4) | PNY_CMD_GET_STATUS), 0u, 0u};
        frame[3] = crc8(frame, 3u);
        stream_stamp_us_ = now;
        stream_waiting_ = true;
        uart().write(frame, sizeof(frame));
    }

    // Stop every ESC this host commanded, then forget them.
    void stopOutputs()
    {
        setBaud(PENNY_BAUD_APP, false);
        for (uint8_t address = 0u; address < 16u; address++) {
            if ((addresses_ & (1u << address)) == 0u) continue;
            uint8_t frame[] = {PNY_FRAME_START, (uint8_t)((address << 4) | PNY_CMD_SEND_DUTY), 2u, 0u, 0u, 0u};
            frame[5] = crc8(frame, 5u);
            uart().write(frame, sizeof(frame));
        }
        addresses_ = 0u;
        stopAux();
    }

    void setBaud(uint8_t baud, bool force)
    {
        if (!force && baud == baud_) return;
        uart().flush();
        uart().end();
        uart().setRxBufferSize(1024);
        uart().begin(baud == PENNY_BAUD_BOOT ? PENNYESC_BAUD_UPDATE : PENNYESC_BAUD_FAST, SERIAL_8N1, rx_, tx_);
        gpio_pullup_en((gpio_num_t)rx_);
        while (uart().available() > 0) uart().read();
        baud_ = baud;
        frame_length_ = 0u;
    }

    void driveGround()
    {
        if (gnd_ < 0) return;
        pinMode(gnd_, OUTPUT);
        digitalWrite(gnd_, LOW);
    }

    uint8_t changePins(int rx, int tx, int gnd)
    {
        if (!validPins(rx, tx, gnd) || (aux_enabled_ && (aux_pin_ == rx || aux_pin_ == tx || aux_pin_ == gnd))) {
            return PENNY_BRIDGE_INVALID;
        }
        Preferences settings;
        if (!settings.begin("pennyesc", false)) return PENNY_BRIDGE_SAVE_FAILED;
        size_t saved = settings.putUInt("pins", packPins(rx, tx, gnd));
        settings.end();
        if (saved != sizeof(uint32_t)) return PENNY_BRIDGE_SAVE_FAILED;
        uart().flush();
        uart().end();
        pinMode(rx_, INPUT);
        pinMode(tx_, INPUT);
        if (gnd_ >= 0) pinMode(gnd_, INPUT);
        rx_ = rx;
        tx_ = tx;
        gnd_ = gnd;
        driveGround();
        setBaud(baud_, true);
        return PENNY_BRIDGE_OK;
    }

    void stopAux()
    {
        if (!aux_enabled_) return;
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        pinMode(aux_pin_, INPUT);
        aux_enabled_ = false;
    }

    // 200 Hz servo PWM. A pulse of 0 turns the output off.
    uint8_t setAux(int pin, int pulse)
    {
        if (pulse == 0) {
            stopAux();
            return PENNY_BRIDGE_OK;
        }
        if (!validPin(pin, true) || pin == rx_ || pin == tx_ || pin == gnd_ || pulse < 1000 || pulse > 2000) {
            return PENNY_BRIDGE_INVALID;
        }
        const uint32_t duty = (uint32_t)pulse * (1u << 14) / 5000u;
        if (!aux_enabled_ || pin != aux_pin_) {
            stopAux();
            ledc_timer_config_t timer = {};
            timer.speed_mode = LEDC_LOW_SPEED_MODE;
            timer.duty_resolution = LEDC_TIMER_14_BIT;
            timer.timer_num = LEDC_TIMER_0;
            timer.freq_hz = 200;
            timer.clk_cfg = LEDC_AUTO_CLK;
            ledc_channel_config_t channel = {};
            channel.gpio_num = pin;
            channel.speed_mode = LEDC_LOW_SPEED_MODE;
            channel.channel = LEDC_CHANNEL_0;
            channel.timer_sel = LEDC_TIMER_0;
            channel.duty = duty;
            if (ledc_timer_config(&timer) != ESP_OK || ledc_channel_config(&channel) != ESP_OK) {
                pinMode(pin, INPUT);
                return PENNY_BRIDGE_PWM_FAILED;
            }
        } else if (ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty) != ESP_OK
                   || ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0) != ESP_OK) {
            stopAux();
            return PENNY_BRIDGE_PWM_FAILED;
        }
        aux_pin_ = pin;
        aux_pulse_ = pulse;
        aux_enabled_ = true;
        return PENNY_BRIDGE_OK;
    }

    static const char *boardName()
    {
#if defined(ARDUINO_XIAO_ESP32S3)
        return "xiao-esp32s3";
#elif defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2)
        return "qtpy-esp32s3";
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
        return "esp32-h2";
#else
        return ARDUINO_BOARD;
#endif
    }

    static bool validPin(int pin, bool output)
    {
        if (pin < 0 || !GPIO_IS_VALID_GPIO(pin) || (output && !GPIO_IS_VALID_OUTPUT_GPIO(pin))) return false;
#if defined(ARDUINO_XIAO_ESP32S3)
        return (pin >= 1 && pin <= 9) || pin == 43 || pin == 44;
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
        // Reserve native USB, flash, the crystal and the USB-UART receive line.
        return pin <= 5 || (pin >= 8 && pin <= 12) || pin == 22 || pin == 24 || pin == 25;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
        // Keep native USB, flash and PSRAM pins available to their peripherals.
        return pin <= 18 || pin == 21 || pin >= 35;
#else
        return false;
#endif
    }

    static bool validGround(int pin)
    {
        if (!validPin(pin, true)) return false;
        // The external ground wire also holds this pin low during reset.
#if defined(CONFIG_IDF_TARGET_ESP32H2)
        return pin != 8 && pin != 9 && pin != 25;
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
        return pin != 0 && pin != 3 && pin != 45 && pin != 46;
#else
        return false;
#endif
    }

    static bool validPins(int rx, int tx, int gnd)
    {
        return validPin(rx, false) && validPin(tx, true) && (gnd == -1 || validGround(gnd))
            && rx != tx && rx != gnd && tx != gnd;
    }

    static uint32_t packPins(int rx, int tx, int gnd)
    {
        return (uint32_t)(rx + 1) | ((uint32_t)(tx + 1) << 8) | ((uint32_t)(gnd + 1) << 16);
    }

    HardwareSerial &uart() const { return *uart_; }

    UsbLink usb_;
    Link *owner_ = 0;
    HardwareSerial *uart_ = &Serial1;
    int rx_ = 12, tx_ = 13, gnd_ = -1;
    int aux_pin_ = 11, aux_pulse_ = 1500;
    bool aux_enabled_ = false;
    uint8_t baud_ = 0xffu;
    uint16_t addresses_ = 0u;
    uint32_t last_uart_ms_ = 0u;
    uint8_t usb_buf_[MAX_PACKET + MAX_PACKET / 254u + 2u];
    size_t usb_length_ = 0u;
    uint8_t stream_address_ = 1u;
    uint32_t stream_period_us_ = 0u, next_stream_us_ = 0u, stream_stamp_us_ = 0u;
    bool stream_waiting_ = false;
    uint8_t frame_[PNY_FRAME_MAX_PAYLOAD + 4u];
    size_t frame_length_ = 0u;
    uint8_t batch_[248];
    size_t batch_length_ = 0u;
    uint32_t batch_ms_ = 0u;
};

#endif
