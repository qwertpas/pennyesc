import {SerialConnection, TimeoutError} from './serial.js';
import {decodeFrame, unpackStatus} from './protocol.js';

export const BLE_SERVICE = 'fa7d1001-41d2-4e87-8b47-79f2ac530001';
export const BLE_WRITE = 'fa7d1002-41d2-4e87-8b47-79f2ac530001';
export const BLE_REPLY = 'fa7d1003-41d2-4e87-8b47-79f2ac530001';
export const BLE_TELEMETRY = 'fa7d1004-41d2-4e87-8b47-79f2ac530001';

export class BleConnection extends SerialConnection {
  constructor() {
    super();
    this.guardMs = 0;
    this.samples = [];
  }

  async connect(device) {
    if (SerialConnection.owner) throw new Error('Another connection owns the device');
    if (!device) device = await navigator.bluetooth.requestDevice({filters: [{services: [BLE_SERVICE]}]});
    this.samples = [];
    this.sampleFailure = null;
    this.lastStamp = undefined;
    const connection = this;
    let controller, reply, telemetry, onReply, onTelemetry;
    const onDisconnect = () => controller?.error(new Error('BLE disconnected'));
    const port = {
      async open() {
        device.addEventListener('gattserverdisconnected', onDisconnect);
        const server = await device.gatt.connect();
        const service = await server.getPrimaryService(BLE_SERVICE);
        const input = await service.getCharacteristic(BLE_WRITE);
        reply = await service.getCharacteristic(BLE_REPLY);
        telemetry = await service.getCharacteristic(BLE_TELEMETRY);
        port.readable = new ReadableStream({start(value) { controller = value; }, cancel() { controller = null; }});
        port.writable = new WritableStream({write: bytes => input.writeValueWithoutResponse(bytes)});
        onReply = event => {
          const value = event.target.value;
          controller?.enqueue(new Uint8Array(value.buffer, value.byteOffset, value.byteLength).slice());
        };
        onTelemetry = event => connection.receiveSamples(event.target.value);
        reply.addEventListener('characteristicvaluechanged', onReply);
        telemetry.addEventListener('characteristicvaluechanged', onTelemetry);
        await reply.startNotifications();
        await telemetry.startNotifications();
      },
      async setSignals() {},
      getInfo: () => ({transport: 'BLE', name: device.name}),
      async close() {
        device.removeEventListener('gattserverdisconnected', onDisconnect);
        reply?.removeEventListener('characteristicvaluechanged', onReply);
        telemetry?.removeEventListener('characteristicvaluechanged', onTelemetry);
        device.gatt.disconnect();
        port.readable = port.writable = null;
      },
    };
    try {
      await super.connect(port);
      this._prefix = '';
      return this;
    } catch (error) {
      await port.close();
      throw error;
    }
  }

  async _mode(mode) {
    if (!['app', 'off'].includes(mode)) throw new Error('Firmware flashing requires USB');
  }

  disconnect() {
    this.streaming = false;
    this.sampleFailure = new Error('BLE disconnected');
    this.sampleWake?.();
    return super.disconnect();
  }

  receiveSamples(view) {
    if (!this.streaming) return;
    try {
      for (let offset = 0; offset < view.byteLength;) {
        if (view.byteLength - offset < 8) throw new Error('Incomplete BLE telemetry');
        const stamp = view.getUint32(offset, true);
        const size = view.getUint8(offset + 6) + 4;
        if (offset + 4 + size > view.byteLength) throw new Error('Incomplete BLE telemetry frame');
        const frame = decodeFrame(new Uint8Array(view.buffer, view.byteOffset + offset + 4, size));
        if (frame.command !== 1 || frame.address !== this.streamAddress) throw new Error('Unexpected BLE telemetry frame');
        this.sampleTime = this.lastStamp === undefined ? performance.now() : this.sampleTime + ((stamp - this.lastStamp) >>> 0) / 1000;
        this.lastStamp = stamp;
        this.samples.push({status: unpackStatus(frame.payload), now: this.sampleTime});
        if (this.samples.length > 512) throw new Error('BLE telemetry queue overflow');
        offset += 4 + size;
      }
    } catch (error) { this.sampleFailure = error; }
    this.sampleWake?.();
  }

  async startTelemetry(address, rate) {
    this.samples = [];
    this.sampleFailure = null;
    this.streamAddress = address;
    this.streaming = true;
    await this.shell(`stream ${address} ${rate}`, {expected: '# stream=on'});
  }

  async stopTelemetry() {
    this.streaming = false;
    await this.shell('stream off', {expected: '# stream=off'});
    this.samples = [];
  }

  async nextTelemetry(timeout = 500) {
    if (!this.samples.length && !this.sampleFailure) {
      await new Promise((resolve, reject) => {
        const timer = setTimeout(() => { this.sampleWake = null; reject(new TimeoutError('BLE telemetry timed out')); }, timeout);
        this.sampleWake = () => { clearTimeout(timer); this.sampleWake = null; resolve(); };
      });
    }
    if (this.sampleFailure) throw this.sampleFailure;
    return this.samples.shift();
  }
}
