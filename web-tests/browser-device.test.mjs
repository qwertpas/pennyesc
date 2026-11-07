import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import test from 'node:test';
import {BrowserDevice} from './browser-device.js';
import {FakeBridge} from './serial-device.mjs';
import {SerialConnection, TimeoutError} from '../docs/serial.js';
import {PennyClient} from '../docs/protocol.js';
import {captureCalibration, reduceCapturePoints, solveCapture, uploadCalibration} from '../docs/calibration.js';
import {auxPulse, DutyKeys} from '../docs/keyboard.js';

test('Aux presets and duty keys share serial telemetry without interfering with each other', async () => {
  const fixture = JSON.parse(readFileSync(new URL('./calibration-fixture.json', import.meta.url)));
  const device = new BrowserDevice(fixture);
  const connection = new SerialConnection();
  const bridge = new FakeBridge(bytes => device.uart(bytes), {fragment: 7, delay: 1});
  await connection.connect(bridge.port);
  try {
    const client = new PennyClient(connection, 1);
    const keys = new DutyKeys({value: () => 120, send: duty => client.setDuty(duty), onError(error) { throw error; }});
    keys.enabled = true;
    keys.keyDown({key: '+', code: 'Equal', preventDefault() {}});
    await keys.task;
    for (let key = 1; key <= 9; key++) {
      const pulse = auxPulse({key: String(key)});
      const [status, bridgeState] = await Promise.all([client.status(), connection.setAux(1, pulse)]);
      assert.equal(status.duty, 120);
      assert.deepEqual(bridgeState.aux, {pin: 1, pulse, enabled: true});
    }
    await keys.release();
    assert.equal((await client.status()).duty, 0);
    assert.deepEqual(bridge.aux, {pin: 1, pulse: 2000, enabled: true});
    await assert.rejects(connection.setAux(12, 1500), /Invalid pin/);
    assert.equal((await client.status()).duty, 0);
    await connection.auxOff();
    assert.equal(bridge.aux.enabled, false);
  } finally { await connection.disconnect(); }
});

test('browser harness supports the complete serial calibration workflow and saved pins', async () => {
  const fixture = JSON.parse(readFileSync(new URL('./calibration-fixture.json', import.meta.url)));
  const device = new BrowserDevice(fixture);
  const bridge = new FakeBridge(bytes => device.uart(bytes), {fragment: 13, delay: 1, pins: [7, 44, -1]});
  const port = bridge.port;
  const connection = new SerialConnection();
  await connection.connect(port);
  try {
    assert.deepEqual([connection.state.rx, connection.state.tx], [7, 44]);
    await assert.rejects(new PennyClient(connection, 0).status(5), TimeoutError);
    const client = new PennyClient(connection, 1);
    assert.equal((await client.status()).calibrated, true);
    const pins = await connection.setPins(1, 2, -1);
    assert.deepEqual([pins.rx, pins.tx, pins.gnd], [1, 2, -1]);
    const raw = await captureCalibration(client);
    assert.equal(raw.length, 216);
    const solved = solveCapture(reduceCapturePoints(raw));
    const stored = await uploadCalibration(client, solved);
    assert.equal(stored.blob_crc32, fixture.cases[0].solved.blob_crc32);
    assert.equal(device.stats.sweeps, 6);
    assert.equal(device.stats.pointsRead, 216);
    assert.equal(device.stats.bytesUploaded, 640);
    assert.equal(device.stats.commits, 1);
    assert.deepEqual(bridge.pins, [1, 2, -1]);
  } finally { await connection.disconnect(); }
  assert.equal(port.openCount, 1);
  assert.equal(port.closeCount, 1);
  assert.equal(SerialConnection.owner, null);
});
