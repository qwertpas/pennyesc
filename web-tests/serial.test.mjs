import test from 'node:test';
import assert from 'node:assert/strict';
import { SerialConnection, TimeoutError } from '../docs/serial.js';
import { PennyClient, decodeFrame, encodeFrame, pack } from '../docs/protocol.js';
import { FakePort, appResponse, statusPayload } from './serial-device.mjs';

test('one serial reader serializes simultaneous commands and releases both locks', async () => {
  let outstanding = 0;
  const port = new FakePort(async data => {
    assert.equal(outstanding++, 0);
    await new Promise(resolve => setTimeout(resolve, 1));
    outstanding--;
    return appResponse(data);
  }, { fragment: 3 });
  const connection = new SerialConnection();
  await connection.connect(port);
  try {
    const client = new PennyClient(connection, 2);
    const [status, posvel, duty] = await Promise.all([client.status(), client.posVel(), client.setDuty(-50)]);
    assert.equal(status.x, 123); assert.equal(posvel.velocity_rpm, -30); assert.equal(duty.duty, 50);
    assert.equal(port.writes.length, 3);
    const second = new SerialConnection();
    await assert.rejects(() => second.connect(new FakePort(appResponse)), /owns the port/);
  } finally { await connection.disconnect(); }
  assert.equal(SerialConnection.owner, null); assert.equal(port.closeCount, 1);
  await connection.connect(port); await connection.disconnect();
  assert.equal(port.openCount, 2);
});

test('late partial responses survive timeout without resetting the serial input', async () => {
  const reply = encodeFrame(1, 1, statusPayload());
  let calls = 0;
  const port = new FakePort(() => {
    if (++calls === 1) { port.feed(reply.subarray(0, 7)); return null; }
    return reply.subarray(7);
  });
  const connection = new SerialConnection(); await connection.connect(port);
  try {
    await assert.rejects(() => connection.exchange(1, 1, new Uint8Array(), 5), TimeoutError);
    const payload = await connection.exchange(1, 1, new Uint8Array(), 100);
    assert.deepEqual(payload, statusPayload());
  } finally { await connection.disconnect(); }
});

test('shell waits for escape guard and recognizes mode acknowledgement', async () => {
  const encoder = new TextEncoder(), decoder = new TextDecoder();
  const port = new FakePort(data => {
    if (data[0] === 0xaa) return appResponse(data);
    const command = decoder.decode(data).trim();
    if (command === '!#ping') return encoder.encode('pong\n');
    if (command === '!#bridge app') return encoder.encode('# bridge=app\n');
    if (command === '!#pins') return encoder.encode('# pins board=test rx=7 tx=44 gnd=-1\n');
    if (command === '!#pins 7 7 -1') return encoder.encode('# pins error=invalid\n');
    return encoder.encode('err\n');
  }, { fragment: 2 });
  const connection = new SerialConnection(); await connection.connect(port);
  try {
    await connection.mode('app'); await connection.exchange(0, 1);
    assert.match(await connection.shell('pins'), /rx=7 tx=44/);
    const last = port.writes.at(-1), before = port.writes.at(-2);
    assert.ok(last.time - before.time >= 24, `Escape guard ${last.time - before.time} ms`);
    await assert.rejects(() => connection.shell('pins 7 7 -1'), /error=invalid/);
    await assert.rejects(() => connection.shell('unknown'), /err/);
  } finally { await connection.disconnect(); }
});

test('continuous full-status acquisition exceeds 100 samples/s with fragmented USB replies', async () => {
  const port = new FakePort(data => {
    const { address, command } = decodeFrame(data);
    return encodeFrame(address, command, statusPayload());
  }, { fragment: 7, delay: 1 });
  const connection = new SerialConnection(); await connection.connect(port);
  try {
    const client = new PennyClient(connection, 0);
    const start = performance.now();
    for (let i = 0; i < 250; i++) await client.status();
    const rate = 250000 / (performance.now() - start);
    assert.ok(rate >= 100, `Measured ${rate.toFixed(1)} samples/s`);
    assert.equal(connection._chunks.length, 0); assert.equal(connection._frames.frames.length, 0);
  } finally { await connection.disconnect(); }
});

test('unplug rejects pending exchange and releases port ownership without a UI cleanup handler', async () => {
  const port = new FakePort(() => null);
  const connection = new SerialConnection(); await connection.connect(port);
  const pending = connection.exchange(0, 1);
  const rejected = assert.rejects(pending, /Unplugged/);
  port.controller.error(new Error('Unplugged'));
  await rejected;
  await new Promise(resolve => setTimeout(resolve, 0));
  assert.equal(connection.connected, false);
  assert.equal(SerialConnection.owner, null);
  assert.equal(port.closeCount, 1);
  await connection.connect(new FakePort(appResponse)); await connection.disconnect();
});
