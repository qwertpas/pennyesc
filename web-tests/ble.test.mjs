import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import test from 'node:test';
import {BleConnection, BLE_SERVICE, BLE_WRITE, BLE_REPLY, BLE_TELEMETRY} from '../docs/ble.js';
import {SerialConnection} from '../docs/serial.js';
import {PennyClient, encodeFrame} from '../docs/protocol.js';
import {Telemetry} from '../docs/telemetry.js';
import {BrowserDevice} from './browser-device.js';

const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
const encoder = new TextEncoder(), decoder = new TextDecoder();

class Characteristic extends EventTarget {
  async startNotifications() { return this; }
  notify(bytes) {
    this.value = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    this.dispatchEvent(new Event('characteristicvaluechanged'));
  }
}

function device() {
  const fixture = JSON.parse(readFileSync(new URL('./calibration-fixture.json', import.meta.url)));
  const motor = new BrowserDevice(fixture), writes = [];
  motor.mode = 'app';
  const input = new Characteristic(), reply = new Characteristic(), telemetry = new Characteristic();
  const peripheral = new EventTarget();
  let timer, stamp = 0xffff0000;
  peripheral.name = 'PennyESC';
  peripheral.gatt = {
    connected: false,
    async connect() { this.connected = true; return this; },
    async getPrimaryService(uuid) {
      assert.equal(uuid, BLE_SERVICE);
      return {async getCharacteristic(id) {
        return {[BLE_WRITE]: input, [BLE_REPLY]: reply, [BLE_TELEMETRY]: telemetry}[id];
      }};
    },
    disconnect() {
      clearInterval(timer);
      if (!this.connected) return;
      this.connected = false;
      peripheral.dispatchEvent(new Event('gattserverdisconnected'));
    },
  };
  input.writeValueWithoutResponse = async bytes => {
    writes.push(bytes.slice());
    const text = decoder.decode(bytes).trim();
    let response;
    if (text === 'stream off') {
      clearInterval(timer);
      response = encoder.encode('# stream=off\n');
    } else if (text.startsWith('stream ')) {
      const [, address, hz] = text.split(' ').map(Number);
      assert.equal(address, 1); assert.equal(hz, 200);
      clearInterval(timer);
      timer = setInterval(() => {
        const records = [];
        for (let i = 0; i < 3; i++) {
          const frame = encodeFrame(1, 1, motor.status());
          const record = new Uint8Array(4 + frame.length);
          new DataView(record.buffer).setUint32(0, stamp >>> 0, true);
          stamp += 5000;
          record.set(frame, 4); records.push(...record);
        }
        telemetry.notify(new Uint8Array(records));
      }, 15);
      response = encoder.encode('# stream=on\n');
    } else response = motor.write(bytes);
    if (response) for (let offset = 0; offset < response.length; offset += 13) reply.notify(response.subarray(offset, offset + 13));
  };
  return {peripheral, motor, writes, telemetry};
}

test('BLE batches 200 Hz telemetry with device timestamps while controls and Aux share the link', async () => {
  const {peripheral, motor, writes} = device();
  const connection = new BleConnection(), samples = [], rates = [], errors = [];
  await connection.connect(peripheral);
  const client = new PennyClient(connection, 1);
  const telemetry = new Telemetry(client, {onSample: (status, now) => samples.push({status, now}),
    onRate: rate => rates.push(rate), onError: error => errors.push(error)});
  try {
    await connection.mode('app');
    await assert.rejects(connection.mode('upload'), /requires USB/);
    assert.equal((await client.status()).calibrated, true);
    telemetry.start();
    await pause(100);
    await client.setDuty(123);
    await connection.shell('aux 1 1750', {expected: line => line.startsWith('# aux ')});
    await pause(1000);
    await telemetry.stop();
    assert.ok(samples.length >= 180, `Received ${samples.length} samples`);
    assert.ok(rates.some(rate => rate >= 170 && rate <= 220), rates.join(','));
    for (let i = 1; i < samples.length; i++) assert.ok(Math.abs(samples[i].now - samples[i - 1].now - 5) < 0.001);
    assert.ok(samples.some(sample => sample.status.duty === 123));
    assert.deepEqual(motor.aux, {pin: 1, pulse: 1750, enabled: 1});
    assert.equal(writes.filter(bytes => bytes[0] === 0xaa && (bytes[1] & 15) === 1).length, 1);
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
  const pending = connection.shell('pins', {expected: 'never'});
  await pause(5);
  peripheral.gatt.disconnect();
  await assert.rejects(pending, /BLE disconnected/);
  await disconnected;
  await connection.disconnect();
  assert.equal(SerialConnection.owner, null);
  assert.equal(connection.connected, false);
});

test('malformed BLE telemetry ends acquisition with an explicit error', async () => {
  const fake = device(), connection = new BleConnection(), errors = [];
  await connection.connect(fake.peripheral);
  const telemetry = new Telemetry(new PennyClient(connection, 1), {onSample() {}, onRate() {}, onError: error => errors.push(error)});
  try {
    telemetry.start();
    await pause(8);
    fake.telemetry.notify(Uint8Array.of(1, 2, 3));
    await telemetry.task;
    assert.match(errors[0].message, /Incomplete BLE telemetry/);
    assert.equal(telemetry.running, false);
  } finally { await telemetry.stop(); await connection.disconnect(); }
});
