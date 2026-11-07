#pragma once

#include "pennyesc_bridge.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <host/ble_hs_adv.h>

static constexpr char PENNY_BLE_SERVICE[] = "fa7d1001-41d2-4e87-8b47-79f2ac530001";
static constexpr char PENNY_BLE_WRITE[] = "fa7d1002-41d2-4e87-8b47-79f2ac530001";
static constexpr char PENNY_BLE_REPLY[] = "fa7d1003-41d2-4e87-8b47-79f2ac530001";

// Adds BLE as a second packet link: one GATT write or notification per packet.
// BLE callbacks only queue packets; poll() owns the UART. USB takes priority:
// a USB packet disconnects BLE, and BLE is refused for one second afterwards.
class PennyBleBridge : public PennyBridge, private BLEServerCallbacks, private BLECharacteristicCallbacks {
public:
    void beginBle()
    {
        packets_ = xQueueCreate(8, sizeof(Packet));
        BLEDevice::init("PennyESC");
        ble_gap_set_prefered_default_le_phy(BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK);
        BLEDevice::setMTU(247);
        server_ = BLEDevice::createServer();
        server_->setCallbacks(this);
        BLEService *service = server_->createService(PENNY_BLE_SERVICE);
        ble_.reply = service->createCharacteristic(PENNY_BLE_REPLY, BLECharacteristic::PROPERTY_NOTIFY);
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

    void poll() override
    {
        PennyBridge::poll();
        if (lost_) {
            lost_ = false;
            if (owner_ == &ble_) {
                stopStream();
                stopOutputs();
                owner_ = 0;
            }
        }
        Packet packet;
        while (packets_ && xQueueReceive(packets_, &packet, 0) == pdTRUE) {
            handlePacket(ble_, packet.bytes, packet.length);
        }
    }

protected:
    bool accept(Link &link) override
    {
        if (&link == &ble_ && usb_seen_ && millis() - last_usb_ms_ < 1000u) return false;
        if (&link != owner_ && owner_ == &ble_) stopOutputs();
        return PennyBridge::accept(link);
    }

    void onUsbPacket() override
    {
        usb_seen_ = true;
        last_usb_ms_ = millis();
        if (ble_.connected && !kicked_) {
            kicked_ = true;
            server_->disconnect(server_->getConnId());
        }
    }

    uint8_t startStream(uint8_t address, uint8_t hz) override
    {
        if (owner_ == &ble_ && hz != 0u && ble_.mtu < 75u) return PENNY_BRIDGE_INVALID;
        return PennyBridge::startStream(address, hz);
    }

private:
    struct Packet { uint8_t bytes[MAX_PACKET]; size_t length; };

    class BleLink : public Link {
    public:
        BLECharacteristic *reply = nullptr;
        volatile bool connected = false;
        volatile uint16_t mtu = 23;
        void send(const uint8_t *packet, size_t length) override
        {
            if (!connected) return;
            reply->setValue(const_cast<uint8_t *>(packet), length);
            reply->notify();
        }
        size_t maxPacket() const override { return mtu - 3u; }
    };

    void onConnect(BLEServer *, ble_gap_conn_desc *) override
    {
        kicked_ = false;
        ble_.connected = true;
    }
    void onDisconnect(BLEServer *) override
    {
        ble_.connected = false;
        ble_.mtu = 23;
        lost_ = true;
        xQueueReset(packets_);
        server_->startAdvertising();
    }
    void onMtuChanged(BLEServer *server, ble_gap_conn_desc *desc, uint16_t mtu) override
    {
        ble_.mtu = mtu;
        server->updateConnParams(desc->conn_handle, 12, 12, 0, 200);
        ble_gap_set_data_len(desc->conn_handle, 251, 2120);
    }
    void onWrite(BLECharacteristic *input) override
    {
        Packet packet = {};
        packet.length = input->getLength();
        if (packet.length == 0u || packet.length > sizeof(packet.bytes)) return;
        memcpy(packet.bytes, input->getData(), packet.length);
        xQueueSend(packets_, &packet, 0);
    }

    BLEServer *server_ = nullptr;
    BleLink ble_;
    QueueHandle_t packets_ = nullptr;
    volatile bool lost_ = false;
    bool kicked_ = false, usb_seen_ = false;
    uint32_t last_usb_ms_ = 0u;
};
