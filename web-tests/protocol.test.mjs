import test from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { CMD, DUTY_LIMIT, FrameReader, PennyClient, checkDuty, checkClip, decodeFrame, encodeFrame, pack, unpack, unpackStatus } from '../docs/protocol.js';
import { statusPayload } from './serial-device.mjs';

test('frames match Python CRC8 wire format for every address and command', () => {
  const python = spawnSync('python3.11', ['-c', `import json
def crc8(data):
 c=0
 for b in data:
  c^=b
  for _ in range(8): c=((c<<1)^(7 if c&128 else 0))&255
 return c
rows=[]
for a in range(16):
 for cmd in range(16):
  b=bytes([170,a*16+cmd,4,170,0,255,10]);rows.append(list(b+bytes([crc8(b)])))
print(json.dumps(rows))`], { encoding: 'utf8' });
  assert.equal(python.status, 0, python.stderr);
  const rows = JSON.parse(python.stdout);
  for (let address = 0; address < 16; address++) for (let command = 0; command < 16; command++) {
    const payload = new Uint8Array([170, 0, 255, 10]);
    const encoded = encodeFrame(address, command, payload);
    assert.deepEqual([...encoded], rows[address * 16 + command]);
    assert.deepEqual(decodeFrame(encoded), { address, command, payload });
  }
  assert.throws(() => encodeFrame(16, 1), /Address/);
  assert.throws(() => encodeFrame(0, 16), /Command/);
  assert.throws(() => encodeFrame(0, 1, new Uint8Array(65)), /64/);
});

test('fragmented stream recovers valid frames inside corrupted data', () => {
  const reader = new FrameReader();
  const fast = encodeFrame(2, 14, new Uint8Array([1, 2, 3]));
  const status = encodeFrame(2, 1, new Uint8Array([5, 6]));
  const stream = new Uint8Array([0, 0xaa, 0x21, 12, 0, ...fast, ...status]);
  for (const byte of stream) reader.feed(new Uint8Array([byte]));
  assert.deepEqual(reader.take(2, 1), new Uint8Array([5, 6]));
  assert.deepEqual(reader.take(2, 14), new Uint8Array([1, 2, 3]));
  assert.ok(reader.crcErrors > 0);
});

test('full status retains all diagnostics and derives physical units', () => {
  const status = unpackStatus(statusPayload());
  assert.equal(status.x, 123); assert.equal(status.y, -456);
  assert.equal(status.position_rad, Math.PI * 2);
  assert.equal(status.velocity_rpm, -30);
  assert.equal(status.tmag_sample_count, 13);
  assert.equal(status.tmag_sample_dt_us, 250);
  assert.equal(status.isr_overrun_count, 8);
  assert.equal(status.uart_overrun_errors, 12);
  assert.equal(status.calibrated, true);
  assert.throws(() => unpackStatus(new Uint8Array(59)), /length/);
});

test('commands use fixed point gains, signed units, and no-reply trajectory frames', async () => {
  const writes = [];
  const connection = {
    exchange: async (...args) => { writes.push(args); return statusPayload(); },
    send: async (...args) => { writes.push(args); },
  };
  const client = new PennyClient(connection, 3);
  await client.setControl({ kp: 100, kd: 0.1, kv: 0.06, kf: -12, clip: 150, deadband_turn32: 100 });
  assert.equal(writes[0][0], 3); assert.equal(writes[0][1], CMD.SET_CONTROL);
  assert.deepEqual(unpack('hhhhhH', writes[0][2]), [25600, 26, 15, -12, 150, 100]);
  await client.setVelocityRpm(-60);
  assert.deepEqual(unpack('i', writes[1][2]), [-65536]);
  await client.sendTrajectory(Math.PI * 2, -Math.PI, 10);
  assert.equal(writes[2][1], CMD.SEND_POSITION);
  assert.deepEqual(unpack('iih', writes[2][2]), [65536, -32768, 10]);
  assert.throws(() => pack('h', 32768), /range/);
  assert.throws(() => pack('I', -1), /range/);
});

test('duty limits cover every client write path before frames reach the connection', async () => {
  const writes = [];
  const connection = {
    exchange: async (...args) => { writes.push(args); return statusPayload(); },
    send: async (...args) => { writes.push(args); },
  };
  const client = new PennyClient(connection, 1);
  assert.equal(DUTY_LIMIT, 799);
  const dutyPaths = [value => client.setDuty(value), value => client.sendDuty(value),
    value => client.sendTrajectory(0, 0, value), value => client.setControl({kf: value}),
    value => client.setControl({kf: value, brushed: true})];
  for (const value of [-800, 800, -1000, 1000, 0.5, NaN, Infinity, '799', null]) {
    for (const action of dutyPaths) assert.throws(() => action(value), /799/, String(value));
    assert.throws(() => checkDuty(value), /799/);
    assert.equal(writes.length, 0);
  }
  for (const value of [-1, -799, -800, 800, 0.5, NaN, Infinity, '799', null]) {
    assert.throws(() => client.setControl({clip: value}), /799/);
    assert.throws(() => client.setControl({clip: value, brushed: true}), /799/);
    assert.throws(() => checkClip(value), /799/);
    assert.equal(writes.length, 0);
  }
  for (const value of [-799, 0, 799]) {
    assert.equal(checkDuty(value), value);
    for (const action of dutyPaths) await action(value);
  }
  await client.setControl({clip: 0});
  await client.setControl({clip: 799});
  assert.equal(writes.length, 17);
  assert.equal(checkClip(0), 0);
  assert.equal(checkClip(799), 799);
  // Generic signed wire packing remains available for other protocol fields.
  assert.deepEqual(unpack('h', pack('h', -32768)), [-32768]);
  assert.deepEqual(unpack('h', pack('h', 32767)), [32767]);
});
