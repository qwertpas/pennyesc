import assert from 'node:assert/strict';
import test from 'node:test';
import {LiveData, Telemetry} from '../docs/telemetry.js';
import {PennyClient, decodeFrame, encodeFrame, pack} from '../docs/protocol.js';
import {SerialConnection, TimeoutError} from '../docs/serial.js';

const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
const statusPayload = sample => pack('BBBBhhhHiihHHHHHHIIIIIIH',
  0, 1, 17, 0, sample, -sample, 300, 1234, sample * 100, 65536, 0, 0,
  10, 11, 12, 13, 14, 0, 0, 0, 0, 0, sample, 1000);

class Port {
  constructor() {
    this.replies = 0;
    this.pending = 0;
    this.maxPending = 0;
    this.readable = new ReadableStream({start: controller => { this.input = controller; }});
    this.writable = new WritableStream({write: async bytes => {
      const request = decodeFrame(bytes);
      assert.equal(request.command, 1);
      this.pending++;
      this.maxPending = Math.max(this.maxPending, this.pending);
      await pause(1);
      const frame = encodeFrame(request.address, request.command, statusPayload(++this.replies));
      this.input.enqueue(frame.subarray(0, 5));
      this.input.enqueue(frame.subarray(5));
      this.pending--;
    }});
  }
  async open(options) { assert.equal(options.baudRate, 115200); }
  async setSignals() {}
  async close() {}
}

test('200 Hz acquisition stores distinct decoded serial responses and stops without overlap', async () => {
  const port = new Port(), connection = new SerialConnection();
  await connection.connect(port);
  const client = new PennyClient(connection, 2), samples = [], rates = [], errors = [];
  const telemetry = new Telemetry(client, {rate: 200,
    onSample: status => samples.push(status), onRate: rate => rates.push(rate), onError: error => errors.push(error)});
  try {
    const start = performance.now();
    telemetry.start();
    telemetry.start();
    await pause(1100);
    await telemetry.stop();
    const elapsed = performance.now() - start;
    assert.ok(samples.length * 1000 / elapsed >= 100, `${samples.length} replies in ${elapsed.toFixed(0)} ms`);
    assert.ok(samples.length * 1000 / elapsed <= 225);
    assert.equal(samples.length, port.replies);
    assert.deepEqual(samples.map(status => status.x), Array.from({length: samples.length}, (_, i) => i + 1));
    assert.ok(samples.every(status => status.y === -status.x && status.velocity_rpm === 60));
    assert.equal(port.maxPending, 1);
    assert.ok(rates.some(rate => rate >= 100 && rate <= 225));
    assert.equal(rates.at(-1), 0);
    assert.deepEqual(errors, []);
    const count = samples.length;
    await pause(20);
    assert.equal(samples.length, count);
    telemetry.start();
    await pause(30);
    await telemetry.stop();
    assert.ok(samples.length > count);
    assert.equal(port.maxPending, 1);
  } finally {
    await telemetry.stop();
    await connection.disconnect();
  }
});

test('a status timeout ends acquisition and reports the error once', async () => {
  const failure = new TimeoutError(), errors = [], rates = [], samples = [];
  let calls = 0;
  const client = {async status(timeout) {
    assert.equal(timeout, 150);
    calls++;
    throw failure;
  }};
  const telemetry = new Telemetry(client, {onSample: s => samples.push(s), onRate: r => rates.push(r), onError: e => errors.push(e)});
  telemetry.start();
  await telemetry.task;
  assert.equal(telemetry.running, false);
  assert.equal(calls, 1);
  assert.deepEqual(samples, []);
  assert.deepEqual(errors, [failure]);
  assert.equal(rates.at(-1), 0);
  await telemetry.stop();
});

test('stop waits for an in-flight reply before the connection can be reused', async () => {
  let reply;
  const status = {x: 1}, samples = [];
  const client = {status: () => new Promise(resolve => { reply = resolve; })};
  const telemetry = new Telemetry(client, {onSample: s => samples.push(s), onRate() {}, onError(error) { throw error; }});
  telemetry.start();
  await Promise.resolve();
  let stopped = false;
  const stop = telemetry.stop().then(() => { stopped = true; });
  await Promise.resolve();
  assert.equal(stopped, false);
  reply(status);
  await stop;
  assert.equal(stopped, true);
  assert.deepEqual(samples, [status]);
});

test('starting while stop is pending waits for the previous acquisition loop', async () => {
  let active = 0, maxActive = 0;
  const replies = [];
  const client = {status() {
    active++;
    maxActive = Math.max(maxActive, active);
    return new Promise(resolve => replies.push(() => { active--; resolve({}); }));
  }};
  const telemetry = new Telemetry(client, {onSample() {}, onRate() {}, onError(error) { throw error; }});
  telemetry.start();
  await Promise.resolve();
  const stopping = telemetry.stop();
  telemetry.start();
  await Promise.resolve();
  assert.equal(active, 1);
  assert.equal(replies.length, 1);
  replies.shift()();
  await stopping;
  await Promise.resolve();
  assert.equal(active, 1);
  assert.equal(maxActive, 1);
  const stopped = telemetry.stop();
  replies.shift()();
  await stopped;
  assert.equal(active, 0);
  assert.equal(maxActive, 1);
});

test('live data remains bounded, orders wraparound and exports the visible time window', () => {
  const data = new LiveData(4);
  assert.deepEqual(data.data(), [[], [], [], [], [], []]);
  assert.equal(data.csv(), 'time_s,x,y,z,position_rad,velocity_rpm\n');
  const statuses = Array.from({length: 7}, (_, i) => ({x: i, y: -i, z: i * 2, position_rad: i / 10, velocity_rpm: i * 60}));
  statuses.forEach((status, i) => data.add(status, 5000 + i * 1000));
  assert.equal(data.count, 4);
  assert.equal(data.total, 7);
  assert.equal(data.latest, statuses.at(-1));
  assert.deepEqual(data.data()[0], [3, 4, 5, 6]);
  assert.deepEqual(data.data()[1], [3, 4, 5, 6]);
  assert.deepEqual(data.data(1)[0], [5, 6]);
  assert.deepEqual(data.data(0)[0], [6]);
  assert.equal(data.csv(), 'time_s,x,y,z,position_rad,velocity_rpm\n3,3,-3,6,0.3,180\n4,4,-4,8,0.4,240\n5,5,-5,10,0.5,300\n6,6,-6,12,0.6,360\n');
  data.clear();
  assert.equal(data.count, 0);
  assert.equal(data.total, 0);
  assert.equal(data.latest, null);
  data.add(statuses[0], 20000);
  assert.deepEqual(data.data()[0], [0]);
});

test('invalid rates and ring capacities are rejected before starting', () => {
  for (const rate of [0, -1, NaN, Infinity]) {
    assert.throws(() => new Telemetry({}, {rate}), /Sample rate/);
  }
  for (const capacity of [0, -1, 1.5, NaN, Infinity]) {
    assert.throws(() => new LiveData(capacity), /Sample capacity/);
  }
});
