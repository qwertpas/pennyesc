import assert from 'node:assert/strict';
import test from 'node:test';
import {runCommand, defaultControl, help} from '../docs/commands.js';
import {PennyClient, CMD, pack, unpack} from '../docs/protocol.js';

const statusPayload = (result = 0, position = 123456) => pack('BBBBhhhHiihHHHHHHIIIIIIH',
  result, 1, 17, 0, 100, -200, 300, 1234, position, 65536, 0, 0,
  10, 11, 12, 13, 14, 0, 0, 0, 0, 0, 1234, 1000);

function device(result = 0) {
  const sent = [];
  const connection = {
    async exchange(address, command, payload) {
      sent.push({address, command, payload: [...payload], response: true});
      return statusPayload(result);
    },
    async send(address, command, payload) {
      sent.push({address, command, payload: [...payload], response: false});
    },
  };
  return {client: new PennyClient(connection, 3), sent};
}

test('command modes retain the desktop gains and capture position before enabling control', async () => {
  const {client, sent} = device();
  let result = await runCommand(client, 'mode pos', defaultControl());
  assert.deepEqual(sent.map(call => call.command), [CMD.BRAKE, CMD.SET_VELOCITY, CMD.SET_POSITION, CMD.SET_CONTROL]);
  assert.deepEqual(unpack('i', Uint8Array.from(sent[1].payload)), [0]);
  assert.deepEqual(unpack('i', Uint8Array.from(sent[2].payload)), [123456]);
  assert.deepEqual(unpack('hhhhh', Uint8Array.from(sent[3].payload)), [25600, 26, 0, 0, 150]);
  assert.deepEqual(result.control, {kp: 100, kd: 0.1, kv: 0, kf: 0, clip: 150});
  assert.equal(result.mode, 'pos');
  result = await runCommand(client, 'mode vel', result.control);
  assert.deepEqual(sent.slice(-2).map(call => call.command), [CMD.SET_VELOCITY, CMD.SET_CONTROL]);
  assert.deepEqual(unpack('i', Uint8Array.from(sent.at(-2).payload)), [0]);
  assert.deepEqual(result.control, {kp: 0, kd: 0.1, kv: 0.06, kf: 0, clip: 150});
  assert.equal(result.mode, 'vel');
  result = await runCommand(client, 'mode duty -42', {...result.control, clip: 700});
  assert.deepEqual(result.control, {kp: 0, kd: 0, kv: 0, kf: -42, clip: 700});
  assert.equal(result.mode, 'duty');
});

test('position and velocity commands send the expected signed protocol units', async () => {
  const {client, sent} = device();
  const control = defaultControl();
  await runCommand(client, `pos ${Math.PI}`, control);
  await runCommand(client, `position ${-Math.PI}`, control);
  await runCommand(client, `vel ${Math.PI * 2}`, control);
  await runCommand(client, 'rpm -120', control);
  assert.deepEqual(sent.map(call => call.command), [CMD.SEND_POSITION, CMD.SEND_POSITION, CMD.SET_VELOCITY, CMD.SET_VELOCITY]);
  assert.deepEqual(sent.map(call => unpack('i', Uint8Array.from(call.payload))[0]), [32768, -32768, 65536, -131072]);
  assert.deepEqual(sent.map(call => call.response), [false, false, true, true]);
  assert.ok(sent.every(call => call.address === 3));
});

test('command parsing retains uppercase modes, signed base prefixes and Python rounding', async () => {
  const {client, sent} = device();
  const control = defaultControl();
  assert.equal((await runCommand(client, 'MODE POS', control)).mode, 'pos');
  for (const [text, expected] of [['kf -0x20', -32], ['kf +0b101', 5], ['kf -0o10', -8]]) {
    assert.equal((await runCommand(client, text, control)).control.kf, expected);
  }
  for (const target of [0.5, 1.5, 2.5, -0.5, -1.5, -2.5]) {
    await runCommand(client, `rpm ${target * 60 / 65536}`, control);
  }
  assert.deepEqual(sent.slice(-6).map(call => unpack('i', Uint8Array.from(call.payload))[0]), [0, 2, 2, 0, -2, -2]);
  await runCommand(client, `kp ${0.5 / 256}`, control);
  assert.equal(unpack('hhhhh', Uint8Array.from(sent.at(-1).payload))[0], 0);
  await assert.rejects(runCommand(client, 'duty 010', control), /Invalid integer/);
});

test('gain, feedforward and clipping commands preserve other control fields', async () => {
  const {client, sent} = device();
  let control = {...defaultControl(), clip: 600};
  for (const [command, key, value] of [['kp 1.5', 'kp', 1.5], ['kd 0.25', 'kd', 0.25],
    ['kv 0.5', 'kv', 0.5], ['kf -31', 'kf', -31], ['clip 799', 'clip', 799]]) {
    const result = await runCommand(client, command, control);
    assert.deepEqual(result.control, {...control, [key]: value});
    control = result.control;
  }
  assert.ok(sent.every(call => call.command === CMD.SET_CONTROL));
  assert.deepEqual(unpack('hhhhh', Uint8Array.from(sent.at(-1).payload)), [384, 64, 128, -31, 799]);
  const duty = await runCommand(client, 'duty 200', control);
  assert.deepEqual(duty.control, {kp: 0, kd: 0, kv: 0, kf: 200, clip: 799});
  const brake = await runCommand(client, 'brake', duty.control);
  assert.deepEqual(brake.control, {...defaultControl(), clip: 799});
});

test('status, zero aliases, advance and offline help are retained', async () => {
  const {client, sent} = device();
  for (const command of ['status', 'zero', 'zero_position', 'zeropos', 'advance -30']) {
    assert.equal((await runCommand(client, command, defaultControl())).status.result, 0);
  }
  assert.deepEqual(sent.map(call => call.command), [CMD.GET_STATUS, CMD.ZERO_POSITION, CMD.ZERO_POSITION, CMD.ZERO_POSITION, CMD.SET_ADVANCE]);
  assert.deepEqual(unpack('h', Uint8Array.from(sent.at(-1).payload)), [-30]);
  assert.match((await runCommand(null, 'help', defaultControl())).message, /mode pos\|vel\|duty/);
  assert.match((await runCommand(null, 'control', defaultControl())).message, /clip 150/);
  assert.match(help, /rpm/);
});

test('bad commands and device failures leave the caller control unchanged', async () => {
  const {client, sent} = device(), control = defaultControl();
  for (const command of ['kp', 'duty 1 2', 'kf 1.1', 'clip Infinity', 'mode bad',
    'mode pos 1', 'zero 1', 'advance 1.5', 'vel NaN', 'pos 9999999999999', 'unknown']) {
    await assert.rejects(runCommand(client, command, control), undefined, command);
  }
  assert.equal(sent.length, 0);
  const failed = device(4);
  await assert.rejects(runCommand(failed.client, 'kp 2', control), /Busy|busy/);
  assert.deepEqual(control, defaultControl());
  await assert.rejects(runCommand(null, 'status', control), /Connect first/);
});

test('terminal duty, feedforward and clip limits reject invalid input before any motor command', async () => {
  const {client, sent} = device(), control = defaultControl();
  for (const command of ['duty 800', 'duty -800', 'kf 800', 'kf -800', 'clip 800',
    'clip -1', 'clip -799', 'mode duty 800', 'mode duty -800', 'mode duty 1.5',
    'mode pos 800', 'mode vel 800']) {
    await assert.rejects(runCommand(client, command, control), undefined, command);
    assert.equal(sent.length, 0, command);
    assert.deepEqual(control, defaultControl());
  }
  // Checks belong to command parsing as well as the transport client.
  const calls = [];
  const recordingClient = {async setControl(next) { calls.push(next); return {result: 0}; }};
  await assert.rejects(runCommand(recordingClient, 'kf 800', control), /799/);
  await assert.rejects(runCommand(recordingClient, 'clip -1', control), /799/);
  await assert.rejects(runCommand(recordingClient, 'mode duty -800', control), /799/);
  await assert.rejects(runCommand(recordingClient, 'duty 1', {...control, clip: 800}), /799/);
  await assert.rejects(runCommand(recordingClient, 'kp 1', {...control, kf: -800}), /799/);
  assert.deepEqual(calls, []);
});

test('terminal duty and feedforward accept both 799 boundaries; clip accepts 0 and 799', async () => {
  const {client, sent} = device(), control = defaultControl();
  for (const command of ['duty -799', 'duty 799', 'kf -799', 'kf 799', 'mode duty -799', 'mode duty 799', 'clip 0', 'clip 799']) {
    await runCommand(client, command, control);
  }
  assert.equal(sent.length, 8);
  assert.deepEqual(sent.map(call => unpack('hhhhh', Uint8Array.from(call.payload))[3]), [-799, 799, -799, 799, -799, 799, 0, 0]);
  assert.deepEqual(sent.slice(-2).map(call => unpack('hhhhh', Uint8Array.from(call.payload))[4]), [0, 799]);
});
