#pragma once

#include "pennyesc_arduino_debug.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <host/ble_hs_adv.h>

static constexpr char PENNY_BLE_SERVICE[] = "fa7d1001-41d2-4e87-8b47-79f2ac530001";
static constexpr char PENNY_BLE_WRITE[] = "fa7d1002-41d2-4e87-8b47-79f2ac530001";
static constexpr char PENNY_BLE_REPLY[] = "fa7d1003-41d2-4e87-8b47-79f2ac530001";
static constexpr char PENNY_BLE_TELEMETRY[] = "fa7d1004-41d2-4e87-8b47-79f2ac530001";

// BLE callbacks only queue commands; the main loop owns the ESC UART.
class PennyEscBleBridge : public PennyEscDebugBridge, private BLEServerCallbacks, private BLECharacteristicCallbacks {
public:
    void beginBle()
    {
        commands_ = xQueueCreate(8, sizeof(Command));
        BLEDevice::init("PennyESC");
        ble_gap_set_prefered_default_le_phy(BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK);
        BLEDevice::setMTU(247);
        server_ = BLEDevice::createServer();
        server_->setCallbacks(this);
        BLEService *service = server_->createService(PENNY_BLE_SERVICE);
        reply_ = service->createCharacteristic(PENNY_BLE_REPLY, BLECharacteristic::PROPERTY_NOTIFY);
        samples_ = service->createCharacteristic(PENNY_BLE_TELEMETRY, BLECharacteristic::PROPERTY_NOTIFY);
        BLECharacteristic *input = service->createCharacteristic(PENNY_BLE_WRITE,
            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
        input->setCallbacks(this);
        service->start();

        // Keep the UUID storage alive across NimBLE advertising restarts.
        static const BLEUUID uuid(PENNY_BLE_SERVICE);
        BLEAdvertisementData advertisement;
        advertisement.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
        advertisement.setCompleteServices(uuid);
        advertisement.setName("PennyESC");
        BLEAdvertising *advertising = BLEDevice::getAdvertising();
        advertising->setAdvertisementData(advertisement);
        static ble_hs_adv_fields fields = {};
        fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
        fields.name = reinterpret_cast<const uint8_t *>("PennyESC");
        fields.name_len = 8;
        fields.name_is_complete = 1;
        fields.uuids128 = &uuid.getNative()->u128;
        fields.num_uuids128 = 1;
        fields.uuids128_is_complete = 1;
        ble_gap_adv_set_fields(&fields);
        BLEDevice::startAdvertising();
    }

    bool poll()
    {
        if (Serial.available()) {
            last_usb_ = millis();
            if (ble_owner_) stopBle();
            usb_owner_ = true;
        }
        if (usb_owner_) {
            if (!Serial.isPlugged()) {
                if (bridge_mode_ == ModeUpload || bridge_mode_ == ModeRom) stopAux();
                else stopOutputs();
                usb_owner_ = false;
            } else {
                PennyEscDebugBridge::poll();
                if (connected_) server_->disconnect(server_->getConnId());
                if (bridge_mode_ == ModeShell && millis() - last_usb_ > 1000u) usb_owner_ = false;
                return true;
            }
        }
        // USB suspend can clear isPlugged without closing the host's port.
        // Keep forwarding its pending reply until another owner takes over.
        if (!ble_owner_ && bridge_mode_ != ModeShell) PennyEscDebugBridge::poll();
        if (lost_) {
            lost_ = false;
            if (ble_owner_) stopBle();
        }
        if (!connected_) return true;
        Command command;
        if (xQueueReceive(commands_, &command, 0) == pdTRUE) {
            if (!ble_owner_) resetBridge();
            ble_owner_ = true;
            process(command);
        }
        if (period_ && (int32_t)(micros() - next_sample_) >= 0) {
            uint32_t stamp = micros();
            next_sample_ += period_;
            if ((int32_t)(stamp - next_sample_) >= 0) next_sample_ = stamp + period_;
            uint8_t frame[PNY_FRAME_MAX_PAYLOAD + 4u], length = 0;
            setAddress(stream_address_);
            if (esc_.exchange(PNY_CMD_GET_STATUS, nullptr, 0, frame, length, 3)) {
                size_t count = sizeof(stamp) + length;
                if (sample_bytes_ + count > mtu_ - 3u) flushSamples();
                memcpy(batch_ + sample_bytes_, &stamp, sizeof(stamp));
                memcpy(batch_ + sample_bytes_ + sizeof(stamp), frame, length);
                sample_bytes_ += count;
            }
        }
        if (sample_bytes_ && millis() - last_batch_ >= 15u) flushSamples();
        return true;
    }

private:
    struct Command { uint8_t bytes[96]; size_t length; };
    class Reply : public Stream {
    public:
        uint8_t bytes[1024];
        size_t length = 0;
        int available() override { return 0; }
        int read() override { return -1; }
        int peek() override { return -1; }
        void flush() override {}
        size_t write(uint8_t byte) override {
            if (length == sizeof(bytes)) return 0;
            bytes[length++] = byte;
            return 1;
        }
    } output_;

    void onConnect(BLEServer *, ble_gap_conn_desc *) override { connected_ = true; }
    void onDisconnect(BLEServer *) override
    {
        connected_ = false;
        lost_ = true;
        mtu_ = 23;
        xQueueReset(commands_);
        server_->startAdvertising();
    }
    void onMtuChanged(BLEServer *server, ble_gap_conn_desc *desc, uint16_t mtu) override
    {
        mtu_ = mtu;
        server->updateConnParams(desc->conn_handle, 12, 12, 0, 200);
        ble_gap_set_data_len(desc->conn_handle, 251, 2120);
    }
    void onWrite(BLECharacteristic *input) override
    {
        Command command = {};
        command.length = input->getLength();
        if (!command.length || command.length >= sizeof(command.bytes)) return;
        memcpy(command.bytes, input->getData(), command.length);
        xQueueSend(commands_, &command, 0);
    }

    void resetBridge()
    {
        bridge_mode_ = ModeShell;
        pennyesc_bridge_active = false;
        line_len_ = 0;
        beginApp();
        esc_.clearRx();
    }
    void stopOutputs()
    {
        uint8_t frame[] = {PNY_FRAME_START, uint8_t((address_ << 4) | PNY_CMD_SEND_DUTY), 2, 0, 0, 0};
        frame[5] = PennyEsc::crc8(frame, 5);
        uart().write(frame, sizeof(frame));
        stopAux();
    }
    void stopBle()
    {
        period_ = 0;
        sample_bytes_ = 0;
        stopOutputs();
        xQueueReset(commands_);
        ble_owner_ = false;
        resetBridge();
    }
    void notify(BLECharacteristic *target, const uint8_t *bytes, size_t length)
    {
        while (length && connected_) {
            size_t count = min(length, size_t(mtu_ - 3u));
            target->setValue(bytes, count);
            target->notify();
            bytes += count;
            length -= count;
        }
    }
    void flushSamples()
    {
        if (sample_bytes_) notify(samples_, batch_, sample_bytes_);
        sample_bytes_ = 0;
        last_batch_ = millis();
    }
    void process(const Command &command)
    {
        output_.length = 0;
        if (command.bytes[0] == PNY_FRAME_START) {
            const uint8_t *frame = command.bytes;
            if (command.length < 4 || frame[2] > PNY_FRAME_MAX_PAYLOAD
                || command.length != size_t(frame[2] + 4)
                || PennyEsc::crc8(frame, command.length - 1) != frame[command.length - 1]) return;
            uint8_t cmd = frame[1] & 15;
            if (cmd == PNY_CMD_CAL || cmd == PNY_CMD_DEBUG || cmd == PNY_CMD_ENTER_BOOT || cmd == PNY_CMD_SET_QUIET) {
                output_.println("# error USB required");
            } else {
                setAddress(frame[1] >> 4);
                if (cmd == PNY_CMD_SEND_DUTY || cmd == PNY_CMD_SEND_POSITION) uart().write(frame, command.length);
                else {
                    uint8_t response[PNY_FRAME_MAX_PAYLOAD + 4u], length = 0;
                    if (esc_.exchange(cmd, frame + 3, frame[2], response, length)) {
                        notify(reply_, response, length);
                    }
                }
            }
        } else {
            char text[sizeof(command.bytes) + 1];
            memcpy(text, command.bytes, command.length);
            text[command.length] = '\0';
            char *line = text;
            if (strncmp(line, "!#", 2) == 0) line += 2;
            line[strcspn(line, "\r\n")] = '\0';
            if (strcmp(line, "stream off") == 0) {
                period_ = 0;
                sample_bytes_ = 0;
                output_.println("# stream=off");
            } else if (strncmp(line, "stream ", 7) == 0) {
                int address, hz;
                char extra;
                if (sscanf(line + 7, "%d %d %c", &address, &hz, &extra) != 2
                    || address < 0 || address > 15 || hz < 1 || hz > 200 || mtu_ < 75) {
                    output_.println("# error Invalid stream rate, address or MTU");
                } else {
                    stream_address_ = address;
                    period_ = 1000000u / hz;
                    next_sample_ = micros();
                    sample_bytes_ = 0;
                    last_batch_ = millis();
                    output_.println("# stream=on");
                }
            } else if (strcmp(line, "ping") == 0 || strcmp(line, "pins") == 0
                       || strcmp(line, "aux") == 0 || strncmp(line, "aux ", 4) == 0) {
                usb_ = &output_;
                handleLine(line);
                usb_ = &Serial;
            } else output_.println("# error USB required");
        }
        notify(reply_, output_.bytes, output_.length);
    }

    BLEServer *server_ = nullptr;
    BLECharacteristic *reply_ = nullptr, *samples_ = nullptr;
    QueueHandle_t commands_ = nullptr;
    volatile bool connected_ = false, lost_ = false;
    volatile uint16_t mtu_ = 23;
    bool ble_owner_ = false, usb_owner_ = false;
    uint32_t last_usb_ = 0, period_ = 0, next_sample_ = 0, last_batch_ = 0;
    uint8_t stream_address_ = 1;
    uint8_t batch_[244];
    size_t sample_bytes_ = 0;
};
