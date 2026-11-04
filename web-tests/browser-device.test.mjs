import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import test from 'node:test';
import {BrowserDevice} from './browser-device.js';
import {FakePort} from './serial-device.mjs';
import {SerialConnection, TimeoutError} from '../docs/serial.js';
import {PennyClient} from '../docs/protocol.js';
import {captureCalibration, reduceCapturePoints, solveCapture, uploadCalibration} from '../docs/calibration.js';
import {auxPulse, DutyKeys} from '../docs/keyboard.js';

test('Aux presets and duty keys share serial telemetry without interfering with each other', async () => {
  const fixture = JSON.parse(readFileSync(new URL('./calibration-fixture.json', import.meta.url)));
  const device = new BrowserDevice(fixture);
  const connection = new SerialConnection();
  await connection.connect(new FakePort(bytes => device.write(bytes), {fragment: 7, delay: 1}));
  try {
    await connection.mode('app');
    const client = new PennyClient(connection, 1);
    const keys = new DutyKeys({value: () => 120, send: duty => client.setDuty(duty), onError(error) { throw error; }});
    keys.enabled = true;
    keys.keyDown({key: '+', code: 'Equal', preventDefault() {}});
    await keys.task;
    for (let key = 1; key <= 9; key++) {
      const pulse = auxPulse({key: String(key)});
      const [status, response] = await Promise.all([
        client.status(), connection.shell(`aux 1 ${pulse}`, {expected: line => line.startsWith('# aux ')}),
      ]);
      assert.equal(status.duty, 120);
      assert.match(response, new RegExp(`pulse=${pulse} hz=200 enabled=1`));
    }
    await keys.release();
    assert.equal((await client.status()).duty, 0);
    assert.deepEqual(device.aux, {pin: 1, pulse: 2000, enabled: 1});
    await assert.rejects(connection.shell('aux 7 1500', {expected: line => line.startsWith('# aux ')}), /aux error=invalid/);
    assert.equal((await client.status()).duty, 0);
    await connection.shell('aux off', {expected: line => line.startsWith('# aux ')});
    assert.equal(device.aux.enabled, 0);
  } finally { await connection.disconnect(); }
});

test('browser harness supports the complete serial calibration workflow and saved pins', async () => {
  const fixture = JSON.parse(readFileSync(new URL('./calibration-fixture.json', import.meta.url)));
  const device = new BrowserDevice(fixture);
  const port = new FakePort(bytes => device.write(bytes), {fragment: 13, delay: 1});
  const connection = new SerialConnection();
  await connection.connect(port);
  try {
    assert.match(await connection.shell('pins', {expected: line => line.startsWith('# pins ')}), /rx=7 tx=44/);
    await connection.mode('app');
    await assert.rejects(new PennyClient(connection, 0).status(5), TimeoutError);
    const client = new PennyClient(connection, 1);
    assert.equal((await client.status()).calibrated, true);
    assert.match(await connection.shell('pins 1 2 -1', {expected: line => line.startsWith('# pins ')}), /rx=1 tx=2 gnd=-1/);
    const raw = await captureCalibration(client);
    assert.equal(raw.length, 216);
    const solved = solveCapture(reduceCapturePoints(raw));
    const stored = await uploadCalibration(client, solved);
    assert.equal(stored.blob_crc32, fixture.cases[0].solved.blob_crc32);
    assert.equal(device.stats.sweeps, 6);
    assert.equal(device.stats.pointsRead, 216);
    assert.equal(device.stats.bytesUploaded, 640);
    assert.equal(device.stats.commits, 1);
    assert.deepEqual(device.pins, [1, 2, -1]);
    await connection.mode('off');
  } finally { await connection.disconnect(); }
  assert.equal(port.openCount, 1);
  assert.equal(port.closeCount, 1);
  assert.equal(SerialConnection.owner, null);
});
