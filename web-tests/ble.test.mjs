import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import test from 'node:test';
import {BleConnection, BLE_SERVICE, BLE_WRITE, BLE_REPLY} from '../docs/ble.js';
import {PACKET, SerialConnection, encodePacket} from '../docs/serial.js';
import {PennyClient, encodeFrame} from '../docs/protocol.js';
import {Telemetry} from '../docs/telemetry.js';
import {BrowserDevice} from './browser-device.js';
import {FakeBridge} from './serial-device.mjs';

const pause = ms => new Promise(resolve => setTimeout(resolve, ms));

class Characteristic extends EventTarget {
  async startNotifications() { return this; }
  notify(bytes) {
    this.value = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    this.dispatchEvent(new Event('characteristicvaluechanged'));
  }
}

// The bridge model, with packets carried by GATT writes and notifications.
// Telemetry arrives in batches of three samples stamped 5 ms apart.
class BleBridge extends FakeBridge {
  constructor(uart, reply) { super(uart); this.reply = reply; this.stamp = 0xffff0000; }
  send(type, payload) { this.reply.notify(encodePacket(type, payload)); }
  async handle(packet) {
    if (packet[0] !== PACKET.STREAM) return super.handle(packet);
    this.stopStream();
    if (packet[2]) {
      this.stream = setInterval(() => {
        const records = [];
        for (let i = 0; i < 3; i++) {
          const frame = this.uart(encodeFrame(packet[1], 1));
          const record = new Uint8Array(4 + frame.length);
          new DataView(record.buffer).setUint32(0, this.stamp >>> 0, true);
          this.stamp += 5000;
          record.set(frame, 4); records.push(...record);
        }
        this.send(PACKET.TELEMETRY, records);
      }, 15);
    }
    this.state();
  }
}

function device() {
  const fixture = JSON.parse(readFileSync(new URL('./calibration-fixture.json', import.meta.url)));
  const motor = new BrowserDevice(fixture);
  const input = new Characteristic(), reply = new Characteristic();
  const bridge = new BleBridge(bytes => motor.uart(bytes), reply);
  const peripheral = new EventTarget();
  peripheral.name = 'PennyESC';
  peripheral.gatt = {
    connected: false,
    async connect() { this.connected = true; return this; },
    async getPrimaryService(uuid) {
      assert.equal(uuid, BLE_SERVICE);
      return {async getCharacteristic(id) { return {[BLE_WRITE]: input, [BLE_REPLY]: reply}[id]; }};
    },
    disconnect() {
      bridge.stopStream();
      if (!this.connected) return;
      this.connected = false;
      peripheral.dispatchEvent(new Event('gattserverdisconnected'));
    },
  };
  input.writeValueWithoutResponse = async packet => { await bridge.handle(packet); };
  return {peripheral, motor, bridge, reply};
}

test('BLE batches 200 Hz telemetry with device timestamps while controls and Aux share the link', async () => {
  const {peripheral, motor, bridge} = device();
  const connection = new BleConnection(), samples = [], rates = [], errors = [];
  await connection.connect(peripheral);
  const client = new PennyClient(connection, 1);
  const telemetry = new Telemetry(client, {onSample: (status, now) => samples.push({status, now}),
    onRate: rate => rates.push(rate), onError: error => errors.push(error)});
  try {
    assert.equal((await client.status()).calibrated, true);
    telemetry.start();
    await pause(100);
    await client.setDuty(123);
    await connection.setAux(1, 1750);
    await pause(1000);
    await telemetry.stop();
    assert.ok(samples.length >= 180, `Received ${samples.length} samples`);
    assert.ok(rates.some(rate => rate >= 170 && rate <= 220), rates.join(','));
    for (let i = 1; i < samples.length; i++) assert.ok(Math.abs(samples[i].now - samples[i - 1].now - 5) < 0.001);
    assert.ok(samples.some(sample => sample.status.duty === 123));
    assert.deepEqual(bridge.aux, {pin: 1, pulse: 1750, enabled: true});
    assert.equal(bridge.uartWrites.filter(write => (write.bytes[1] & 15) === 1).length, 1);
    const count = samples.length;
    await pause(25);
    assert.equal(samples.length, count);
    telemetry.start();
    await pause(25);
    const stopping = telemetry.stop();
    telemetry.start();
    await stopping;
    await pause(40);
    await telemetry.stop();
    assert.ok(samples.length > count);
    await client.brake();
    assert.equal(motor.duty, 0);
    assert.deepEqual(errors, []);
  } finally { await telemetry.stop(); await connection.disconnect(); }
  assert.equal(SerialConnection.owner, null);
  assert.equal(peripheral.gatt.connected, false);
});

test('BLE loss rejects waiting commands and releases the shared connection owner', async () => {
  const {peripheral} = device();
  const connection = new BleConnection();
  await connection.connect(peripheral);
  const disconnected = new Promise(resolve => connection.addEventListener('disconnect', resolve, {once: true}));
  const pending = connection.exchange(9, 1, new Uint8Array(), 5000);
  await pause(5);
  peripheral.gatt.disconnect();
  await assert.rejects(pending, /BLE disconnected/);
  await disconnected;
  await connection.disconnect();
  assert.equal(SerialConnection.owner, null);
  assert.equal(connection.connected, false);
});

test('a damaged BLE telemetry batch is skipped without stopping acquisition', async () => {
  const fake = device(), connection = new BleConnection(), errors = [], samples = [];
  await connection.connect(fake.peripheral);
  const telemetry = new Telemetry(new PennyClient(connection, 1), {onSample: s => samples.push(s), onRate() {}, onError: error => errors.push(error)});
  try {
    telemetry.start();
    await pause(30);
    fake.reply.notify(encodePacket(PACKET.TELEMETRY, [1, 2, 3]));
    await pause(60);
    assert.deepEqual(errors, []);
    assert.equal(telemetry.running, true);
    assert.ok(samples.length >= 6);
  } finally { await telemetry.stop(); await connection.disconnect(); }
});
