import test from 'node:test';
import assert from 'node:assert/strict';
import { BAUD, SerialConnection, TimeoutError, cobsDecode, cobsEncode } from '../docs/serial.js';
import { PennyClient, encodeFrame } from '../docs/protocol.js';
import { FakeBridge, FakePort, appResponse, statusPayload } from './serial-device.mjs';

const pause = ms => new Promise(resolve => setTimeout(resolve, ms));

test('COBS round-trips every byte value and long zero-free runs', () => {
  for (const data of [[], [0], [0, 0], [1, 2, 0, 3], Array.from({ length: 600 }, (_, i) => i % 255 + 1), Array.from({ length: 300 }, (_, i) => i & 255)]) {
    const encoded = cobsEncode(data);
    assert.ok(!encoded.includes(0));
    assert.deepEqual([...cobsDecode(encoded)], data);
  }
  assert.equal(cobsDecode([5, 1]), null);
});

test('one connection serializes simultaneous commands and releases its port', async () => {
  let outstanding = 0;
  const bridge = new FakeBridge(async data => {
    assert.equal(outstanding++, 0);
    await pause(1);
    outstanding--;
    return appResponse(data);
  }, { fragment: 3 });
  const connection = new SerialConnection();
  await connection.connect(bridge.port);
  try {
    assert.equal(connection.state.board, 'qtpy-esp32s3');
    const client = new PennyClient(connection, 2);
    const [status, posvel, duty] = await Promise.all([client.status(), client.posVel(), client.setDuty(-50)]);
    assert.equal(status.x, 123); assert.equal(posvel.velocity_rpm, -30); assert.equal(duty.duty, 50);
    assert.equal(bridge.uartWrites.length, 3);
    assert.ok(bridge.uartWrites.every(write => write.baud === BAUD.APP));
    await assert.rejects(() => new SerialConnection().connect(new FakeBridge(appResponse).port), /owns the port/);
  } finally { await connection.disconnect(); }
  assert.equal(SerialConnection.owner, null); assert.equal(bridge.port.closeCount, 1);
  await connection.connect(bridge.port); await connection.disconnect();
  assert.equal(bridge.port.openCount, 2);
});

test('a late reply to a timed-out request is never returned for the next request', async () => {
  let calls = 0;
  const bridge = new FakeBridge(async data => {
    if (++calls === 1) { await pause(20); return encodeFrame(1, 1, statusPayload(1)); }
    return encodeFrame(1, 1, statusPayload(2));
  });
  const connection = new SerialConnection(); await connection.connect(bridge.port);
  try {
    await assert.rejects(() => connection.exchange(1, 1, new Uint8Array(), 5), TimeoutError);
    await pause(40);
    const status = await new PennyClient(connection, 1).status();
    assert.equal(status.position_turn32, 2);
  } finally { await connection.disconnect(); }
});

test('bridge commands return its state and explain failures', async () => {
  const bridge = new FakeBridge(appResponse, { fragment: 2 });
  const connection = new SerialConnection(); await connection.connect(bridge.port);
  try {
    assert.deepEqual([connection.state.rx, connection.state.tx, connection.state.gnd], [12, 13, -1]);
    const pins = await connection.setPins(7, 44, 3);
    assert.deepEqual([pins.rx, pins.tx, pins.gnd], [7, 44, 3]);
    await assert.rejects(() => connection.setPins(7, 7, -1), /Invalid pin/);
    const aux = await connection.setAux(11, 1750);
    assert.deepEqual(aux.aux, { pin: 11, pulse: 1750, enabled: true });
    await assert.rejects(() => connection.setAux(7, 1500), /Invalid pin/);
    assert.equal((await connection.auxOff()).aux.enabled, false);
    // Bridge commands and ESC frames share one link without a mode switch.
    assert.equal((await new PennyClient(connection, 1).status()).x, 123);
  } finally { await connection.disconnect(); }
});

test('connecting keeps greeting a bridge that is still starting', async () => {
  const bridge = new FakeBridge(appResponse, { ready: 900 });
  const connection = new SerialConnection(); await connection.connect(bridge.port);
  try { assert.equal((await new PennyClient(connection, 1).status()).x, 123); }
  finally { await connection.disconnect(); }
  assert.ok(bridge.commands.length >= 2);
});

test('old shell firmware and other versions get a flash-bridge message', async () => {
  const shell = new FakePort(() => new TextEncoder().encode('err\n'));
  await assert.rejects(() => new SerialConnection().connect(shell), /Flash the bridge firmware/);
  assert.equal(SerialConnection.owner, null);
  await assert.rejects(() => new SerialConnection().connect(new FakeBridge(appResponse, { version: 3 }).port), /version 3 is not supported/);
});

test('ESC frames and bootloader bytes each carry their baud; replies at another baud are dropped', async () => {
  const bridge = new FakeBridge((bytes, baud) => baud === BAUD.BOOT ? Uint8Array.of(0x79) : appResponse(bytes));
  const connection = new SerialConnection(); await connection.connect(bridge.port);
  try {
    await connection.transaction(async io => {
      await io.write(Uint8Array.of(0x7f), BAUD.BOOT);
      assert.deepEqual([...await io.read(1, 200)], [0x79]);
      bridge.rx(appResponse(encodeFrame(1, 1)), BAUD.APP);
      await io.write(Uint8Array.of(0x7f), BAUD.BOOT);
      assert.deepEqual([...await io.read(1, 200)], [0x79]);
    });
    assert.equal((await new PennyClient(connection, 1).status()).x, 123);
    assert.deepEqual(bridge.uartWrites.map(write => write.baud), [1, 1, 0]);
  } finally { await connection.disconnect(); }
});

test('continuous full-status exchanges exceed 100 per second with fragmented USB replies', async () => {
  const bridge = new FakeBridge(appResponse, { fragment: 7, delay: 1 });
  const connection = new SerialConnection(); await connection.connect(bridge.port);
  try {
    const client = new PennyClient(connection, 0);
    const start = performance.now();
    for (let i = 0; i < 250; i++) await client.status();
    const rate = 250000 / (performance.now() - start);
    assert.ok(rate >= 100, `Measured ${rate.toFixed(1)} samples/s`);
    assert.equal(connection._chunks.length, 0); assert.equal(connection._frames.frames.length, 0);
  } finally { await connection.disconnect(); }
});

test('unplug rejects a pending exchange and releases the port', async () => {
  const bridge = new FakeBridge(() => null);
  const connection = new SerialConnection(); await connection.connect(bridge.port);
  const events = [];
  connection.addEventListener('disconnect', event => events.push(event.error.message));
  const pending = connection.exchange(0, 1);
  const rejected = assert.rejects(pending, /Unplugged/);
  bridge.port.controller.error(new Error('Unplugged'));
  await rejected;
  await pause(0);
  assert.deepEqual(events, ['Unplugged']);
  assert.equal(connection.connected, false);
  assert.equal(SerialConnection.owner, null);
  assert.equal(bridge.port.closeCount, 1);
  await connection.connect(new FakeBridge(appResponse).port); await connection.disconnect();
});
