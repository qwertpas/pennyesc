import {SerialConnection} from './serial.js';

export const BLE_SERVICE = 'fa7d1001-41d2-4e87-8b47-79f2ac530001';
export const BLE_WRITE = 'fa7d1002-41d2-4e87-8b47-79f2ac530001';
export const BLE_REPLY = 'fa7d1003-41d2-4e87-8b47-79f2ac530001';

// Same packets as USB, one per GATT write or notification instead of COBS.
export class BleConnection extends SerialConnection {
  async _open(device) {
    device ||= await navigator.bluetooth.requestDevice({filters: [{services: [BLE_SERVICE]}]});
    this.device = device;
    this._onDisconnect = () => this._lost(new Error('BLE disconnected'));
    this._onReply = event => {
      const value = event.target.value;
      this._packet(new Uint8Array(value.buffer, value.byteOffset, value.byteLength).slice());
    };
    device.addEventListener('gattserverdisconnected', this._onDisconnect);
    const server = await device.gatt.connect();
    const service = await server.getPrimaryService(BLE_SERVICE);
    this._input = await service.getCharacteristic(BLE_WRITE);
    this._replies = await service.getCharacteristic(BLE_REPLY);
    this._replies.addEventListener('characteristicvaluechanged', this._onReply);
    await this._replies.startNotifications();
    this._writes = Promise.resolve();
    this.port = {getInfo: () => ({transport: 'BLE', name: device.name})};
  }

  // GATT allows one write at a time.
  _transmit(packet) {
    const write = this._writes.then(() => this._input.writeValueWithoutResponse(packet));
    this._writes = write.catch(() => {});
    return write;
  }

  async _shutdown() {
    this.device?.removeEventListener('gattserverdisconnected', this._onDisconnect);
    this._replies?.removeEventListener('characteristicvaluechanged', this._onReply);
    if (this.device?.gatt.connected) this.device.gatt.disconnect();
    this._input = this._replies = null;
  }
}
