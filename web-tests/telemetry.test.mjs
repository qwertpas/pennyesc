import assert from 'node:assert/strict';
import test from 'node:test';
import {LiveData, Telemetry} from '../docs/telemetry.js';
import {PennyClient, decodeFrame, encodeFrame, pack} from '../docs/protocol.js';
import {SerialConnection} from '../docs/serial.js';
import {FakeBridge} from './serial-device.mjs';

const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
const statusPayload = sample => pack('BBBBhhhHiihHHHHHHIIIIIIH',
  0, 1, 17, 0, sample, -sample, 300, 1234, sample * 100, 65536, 0, 0,
  10, 11, 12, 13, 14, 0, 0, 0, 0, 0, sample, 1000);

function streamingBridge({silentUntil = 0} = {}) {
  let replies = 0;
  const bridge = new FakeBridge(bytes => {
    if (performance.now() < silentUntil) return null;
    const request = decodeFrame(bytes);
    return encodeFrame(request.address, request.command, statusPayload(++replies));
  });
  return {bridge, replies: () => replies};
}

async function connected(bridge) {
  const connection = new SerialConnection();
  await connection.connect(bridge.port);
  return connection;
}

test('200 Hz streaming delivers distinct status samples and stops cleanly', async () => {
  const {bridge} = streamingBridge(), connection = await connected(bridge);
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
    assert.ok(samples.length * 1000 / elapsed >= 100, `${samples.length} samples in ${elapsed.toFixed(0)} ms`);
    assert.ok(samples.length * 1000 / elapsed <= 225);
    assert.deepEqual(samples.map(status => status.x), Array.from({length: samples.length}, (_, i) => i + 1));
    assert.ok(samples.every(status => status.y === -status.x && status.velocity_rpm === 60));
    assert.ok(rates.some(rate => rate >= 100 && rate <= 225));
    assert.equal(rates.at(-1), 0);
    assert.deepEqual(errors, []);
    assert.equal(bridge.stream, null);
    const count = samples.length;
    await pause(30);
    assert.equal(samples.length, count);
    telemetry.start();
    await pause(50);
    await telemetry.stop();
    assert.ok(samples.length > count);
    // Controls still work while telemetry streams.
    telemetry.start();
    await pause(20);
    assert.equal((await client.setDuty(40)).result, 0);
    await telemetry.stop();
  } finally {
    await telemetry.stop();
    await connection.disconnect();
  }
});

test('a silent ESC is reported as stale and streaming resumes when it answers', async () => {
  const {bridge} = streamingBridge({silentUntil: performance.now() + 700}), connection = await connected(bridge);
  const samples = [], errors = [];
  let stale = 0;
  const telemetry = new Telemetry(new PennyClient(connection, 1), {onSample: s => samples.push(s), onRate() {},
    onStale: () => stale++, onError: error => errors.push(error)});
  try {
    telemetry.start();
    await pause(1100);
    assert.ok(stale >= 1);
    assert.ok(samples.length > 20);
    assert.equal(telemetry.running, true);
    assert.deepEqual(errors, []);
  } finally {
    await telemetry.stop();
    await connection.disconnect();
  }
});

test('telemetry re-arms the stream after the bridge restarts', async () => {
  const {bridge} = streamingBridge(), connection = await connected(bridge);
  const samples = [], errors = [];
  let stale = 0;
  const telemetry = new Telemetry(new PennyClient(connection, 1), {onSample: s => samples.push(s), onRate() {},
    onStale: () => stale++, onError: error => errors.push(error)});
  try {
    telemetry.start();
    await pause(200);
    bridge.stopStream();
    const before = samples.length;
    await pause(600);
    assert.ok(stale >= 1);
    assert.ok(samples.length > before + 40, `${samples.length - before} samples after restart`);
    assert.deepEqual(errors, []);
  } finally {
    await telemetry.stop();
    await connection.disconnect();
  }
});

test('stop returns promptly while waiting for a silent ESC', async () => {
  const {bridge} = streamingBridge({silentUntil: Infinity}), connection = await connected(bridge);
  const telemetry = new Telemetry(new PennyClient(connection, 1), {onSample() {}, onRate() {}, onError(error) { throw error; }});
  try {
    telemetry.start();
    await pause(50);
    const start = performance.now();
    await telemetry.stop();
    assert.ok(performance.now() - start < 100, `Stop took ${(performance.now() - start).toFixed(0)} ms`);
    assert.equal(bridge.stream, null);
  } finally { await connection.disconnect(); }
});

test('starting while stop is pending leaves exactly one stream running', async () => {
  const {bridge} = streamingBridge(), connection = await connected(bridge);
  const errors = [], samples = [];
  const telemetry = new Telemetry(new PennyClient(connection, 1), {onSample: s => samples.push(s), onRate() {}, onError: e => errors.push(e)});
  try {
    telemetry.start();
    await pause(30);
    const stopping = telemetry.stop();
    telemetry.start();
    await stopping;
    await pause(100);
    assert.notEqual(bridge.stream, null);
    const count = samples.length;
    await telemetry.stop();
    await pause(30);
    assert.equal(bridge.stream, null);
    assert.equal(samples.length, count);
    assert.deepEqual(errors, []);
  } finally { await connection.disconnect(); }
});

test('losing the connection ends streaming with one error', async () => {
  const {bridge} = streamingBridge(), connection = await connected(bridge);
  const errors = [];
  const telemetry = new Telemetry(new PennyClient(connection, 1), {onSample() {}, onRate() {}, onError: e => errors.push(e)});
  telemetry.start();
  await pause(50);
  bridge.port.controller.error(new Error('Unplugged'));
  await telemetry.task;
  assert.equal(telemetry.running, false);
  assert.deepEqual(errors.map(error => error.message), ['Unplugged']);
  bridge.stopStream();
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
