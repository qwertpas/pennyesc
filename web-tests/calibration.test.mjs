import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import test from 'node:test';
import {reduceCapturePoints, solveCapture, parseBlob, captureCalibration,
  uploadCalibration, readCalibration, calibrationPlots, calibratedAngleTurn16,
  circularMeanTurn16} from '../docs/calibration.js';

const fixture = JSON.parse(readFileSync(new URL('./calibration-fixture.json', import.meta.url)));
const points = rows => rows.map(row => Object.fromEntries(fixture.fields.map((key, i) => [key, row[i]])));

test('browser solver produces the Python calibration blobs byte for byte', () => {
  for (const entry of fixture.cases) {
    const reduced = reduceCapturePoints(points(entry.raw));
    assert.deepEqual(reduced, points(entry.reduced), entry.name);
    const solved = solveCapture(reduced), expected = entry.solved;
    for (const key of ['affine_q20', 'angle_lut', 'commutation_alignment_turn16', 'forward_angle_sign', 'blob_crc32']) {
      assert.deepEqual(solved[key], expected[key], `${entry.name}: ${key}`);
    }
    assert.ok(Math.abs(solved.fit_max_error_deg - expected.fit_max_error_deg) < 1e-9);
    assert.ok(Math.abs(solved.sweep_delta_deg - expected.sweep_delta_deg) < 1e-9);
    assert.equal(Buffer.from(solved.blob).toString('hex'), expected.blob, entry.name);
    assert.deepEqual(parseBlob(solved.blob).angle_lut, solved.angle_lut);
    for (const point of reduced) assert.ok(Number.isInteger(calibratedAngleTurn16(point.x, point.y, solved.affine_q20, solved.angle_lut)));
    const plots = calibrationPlots(points(entry.raw), reduced, solved);
    assert.equal(plots.xy.length, 4);
    assert.equal(plots.field.length, 4);
    assert.equal(plots.residual.length, 4);
    assert.ok(plots.residual.every(series => series.y.length === 36));
    assert.ok(plots.residual[3].y.some(error => error !== 0));
  }
});

test('invalid calibration data is rejected before flash writes', () => {
  const raw = points(fixture.cases[0].raw), reduced = reduceCapturePoints(raw);
  assert.throws(() => reduceCapturePoints(raw.slice(1)), /all 216 points/);
  assert.throws(() => solveCapture(reduced.slice(1)), /all 72 points/);
  assert.throws(() => reduceCapturePoints(raw.map((p, i) => i ? p : {...p, sample_spread: 10001})), /spread too high/);
  assert.throws(() => reduceCapturePoints(raw.map((p, i) => i ? p : {...p, sweep_dir: 1})), /direction mismatch/);
  assert.throws(() => solveCapture(reduced.map(p => ({...p, x: 0, y: 0}))), /singular/);
  const blob = solveCapture(reduced).blob;
  blob[12] ^= 1;
  assert.throws(() => parseBlob(blob), /CRC mismatch/);
  const mean = circularMeanTurn16([65530, 4, 8]);
  assert.ok(mean < 16 || mean > 65520);
  assert.throws(() => circularMeanTurn16([0, 32768]), /ambiguous/);
});

test('calibration plots show live raw fields until reduced anchors are available', () => {
  const raw = points(fixture.cases[0].raw).slice(0, 12).reverse();
  const plots = calibrationPlots(raw, [], undefined);
  assert.deepEqual(plots.field[0].x, Array.from({length: 12}, (_, i) => i));
  assert.deepEqual(plots.field[0].y, raw.slice().reverse().map(point => point.x));
  assert.equal(plots.field[0].name, 'forward X');
  assert.equal(plots.field[1].name, 'forward Y');
  assert.equal(plots.residual.length, 0);
});

class Device {
  constructor(raw) {
    this.raw = raw;
    this.sweep = -1;
    this.active = false;
    this.stored = new Uint8Array(640);
    this.offset = 0;
    this.valid = false;
    this.calls = [];
  }
  async status() {
    return {result: 0, flags: this.valid ? 1 : 0, x: 123, y: -456, z: 789};
  }
  async exchange(cmd, payload, timeout) {
    this.calls.push({cmd, payload: [...payload], timeout});
    if (cmd === 9) return Uint8Array.of(0);
    assert.equal(cmd, 4);
    const bytes = new Uint8Array(640), data = new DataView(bytes.buffer);
    switch (payload[0]) {
      case 1:
        assert.equal(this.active, false);
        assert.equal(payload[1], this.sweep + 1 < 3 ? 0 : 1);
        this.sweep++;
        this.active = true;
        return Uint8Array.of(0, 36);
      case 2:
        if (this.active) {
          this.active = false;
          return Uint8Array.of(0, 1, 18, 36);
        }
        return Uint8Array.of(0, 0, 36, 36);
      case 3: {
        const point = this.raw[this.sweep * 36 + payload[1]];
        bytes.set([0, payload[1], point.step_index, point.sweep_dir]);
        for (const [offset, key] of [[4, 'x'], [6, 'y'], [8, 'z'], [14, 'duty']]) data.setInt16(offset, point[key], true);
        data.setUint16(10, point.xy_radius, true);
        data.setUint16(12, point.sample_spread, true);
        return bytes.slice(0, 16);
      }
      case 4: {
        const offset = payload[1] | payload[2] << 8;
        assert.equal(offset, this.offset);
        this.stored.set(payload.subarray(3), offset);
        this.offset += payload.length - 3;
        data.setUint16(1, this.offset, true);
        return bytes.slice(0, 3);
      }
      case 5:
        assert.equal(this.offset, 640);
        parseBlob(this.stored);
        this.valid = true;
        return Uint8Array.of(0, 1);
      case 7:
        bytes[1] = this.valid ? 1 : 0;
        if (this.valid) {
          const solved = parseBlob(this.stored);
          data.setUint16(2, 640, true);
          data.setUint32(4, solved.blob_crc32, true);
          data.setUint16(8, solved.commutation_alignment_turn16, true);
        }
        return bytes.slice(0, 10);
      case 8: {
        const offset = payload[1] | payload[2] << 8, count = payload[3];
        assert.ok(count <= 63);
        bytes.set(this.stored.subarray(offset, offset + count), 1);
        return bytes.slice(0, count + 1);
      }
      default: throw new Error(`Unexpected calibration command ${payload[0]}`);
    }
  }
}

test('capture, reduce, solve, upload, reboot and readback work as one device workflow', async () => {
  const device = new Device(points(fixture.cases[0].raw));
  const progress = [], samples = [];
  assert.equal((await readCalibration(device)).valid, 0);
  const raw = await captureCalibration(device, {onProgress: p => progress.push(p), onSample: p => samples.push(p)});
  assert.deepEqual(raw, device.raw);
  assert.equal(samples.length, 222);
  assert.equal(progress.at(-1).done, 216);
  const solved = solveCapture(reduceCapturePoints(raw));
  const stored = await uploadCalibration(device, solved, {onProgress: p => progress.push(p)});
  assert.equal(stored.valid, 1);
  assert.deepEqual(stored.blob, solved.blob);
  assert.deepEqual(progress.at(-1), {stage: 'verify', done: 1, total: 1});
  assert.equal(device.calls.filter(call => call.payload[0] === 1 && call.cmd === 4).length, 6);
  assert.equal(device.calls.filter(call => call.payload[0] === 3 && call.cmd === 4).length, 216);
});

test('cancellation finishes the current hardware sweep and prevents the next sweep', async () => {
  const device = new Device(points(fixture.cases[0].raw));
  const controller = new AbortController();
  await assert.rejects(captureCalibration(device, {signal: controller.signal,
    onProgress: () => controller.abort()}), {name: 'AbortError'});
  assert.equal(device.sweep, 0);
  assert.equal(device.calls.filter(call => call.payload[0] === 2).length, 2);
  assert.equal(device.calls.filter(call => call.payload[0] === 3).length, 0);
});

test('an incorrect flash acknowledgement prevents commit', async () => {
  const solved = solveCapture(points(fixture.cases[0].reduced));
  const calls = [];
  const client = {async exchange(cmd, payload) {
    calls.push(payload[0]);
    return Uint8Array.of(0, 0, 0);
  }};
  await assert.rejects(uploadCalibration(client, solved), /upload stopped at 0/);
  assert.deepEqual(calls, [4]);
});
