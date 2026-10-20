import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import test from 'node:test';
import {BrowserDevice} from './browser-device.js';
import {FakePort} from './serial-device.mjs';
import {SerialConnection, TimeoutError} from '../docs/serial.js';
import {PennyClient} from '../docs/protocol.js';
import {captureCalibration, reduceCapturePoints, solveCapture, uploadCalibration} from '../docs/calibration.js';

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
