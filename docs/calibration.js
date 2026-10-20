// Calibration format and solver shared with firmware/tools/pennycal.py.
export const CAL_POINTS_PER_SWEEP = 36;
export const CAL_TOTAL_POINTS = 72;
export const CAL_CAPTURE_TOTAL_POINTS = 216;
export const CAL_BLOB_SIZE = 640;
const LUT_SIZE = 256;
const SCALE = 1048576;
const CMD_CAL = 4;
const CAL_START = 1, CAL_STATUS = 2, CAL_READ_POINT = 3;
const CAL_WRITE_BLOB = 4, CAL_COMMIT = 5, CAL_INFO = 7, CAL_READ_BLOB = 8;
const mod = (a, n) => ((a % n) + n) % n;
const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
const view = bytes => new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);

// Python and NumPy round halfway values to even.
function round(value) {
  const low = Math.floor(value), fraction = value - low;
  return fraction === 0.5 ? low + mod(low, 2) : Math.floor(value + 0.5);
}

export function crc32(bytes) {
  let crc = 0xffffffff;
  for (const byte of bytes) {
    crc ^= byte;
    for (let bit = 0; bit < 8; bit++) crc = (crc >>> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
  }
  return (crc ^ 0xffffffff) >>> 0;
}

export function pseudoLookup(x, y) {
  const ax = Math.abs(x), ay = Math.abs(y);
  let octant, ratio;
  if (ax > ay) {
    octant = x >= 0 ? (y >= 0 ? 0 : 7) : (y >= 0 ? 3 : 4);
    ratio = ax ? Math.floor(ay * 8192 / ax) : 0;
  } else {
    octant = y >= 0 ? (x >= 0 ? 1 : 2) : (x >= 0 ? 6 : 5);
    ratio = ay ? Math.floor(ax * 8192 / ay) : 0;
  }
  ratio = Math.min(ratio, 8191);
  if (octant & 1) ratio = 8191 - ratio;
  return [(octant << 5) | (ratio >> 8), ratio & 255];
}

export function applyAffine(x, y, a) {
  // Avoid bitwise shifts: intermediate integer products exceed 32 bits.
  return [Math.floor((x * a[0] + y * a[1] + a[2]) / SCALE),
    Math.floor((x * a[3] + y * a[4] + a[5]) / SCALE)];
}

export function signedTurn16ErrorDeg(measured, target) {
  return (mod(measured - target + 32768, 65536) - 32768) * 360 / 65536;
}

export function calibratedAngleTurn16(x, y, affine, lut) {
  const [index, fraction] = pseudoLookup(...applyAffine(x, y, affine));
  const a = lut[index], b = lut[(index + 1) & 255];
  const delta = mod(b - a + 32768, 65536) - 32768;
  const value = delta * fraction;
  const rounded = Math.sign(value) * Math.floor((Math.abs(value) + 128) / 256);
  return mod(a + rounded, 65536);
}

// The original static residual plots use the LUT bin without interpolation.
export function angleFromXY(x, y, affine, lut) {
  return lut[pseudoLookup(...applyAffine(x, y, affine))[0]];
}

export function circularMeanTurn16(values) {
  if (!values.length) throw new Error('No angles to average');
  let x = 0, y = 0;
  for (const value of values) {
    const angle = mod(value, 65536) * Math.PI * 2 / 65536;
    x += Math.cos(angle);
    y += Math.sin(angle);
  }
  if (Math.abs(x) < 1e-9 && Math.abs(y) < 1e-9) throw new Error('Alignment average is ambiguous');
  return mod(round(mod(Math.atan2(y, x), Math.PI * 2) * 65536 / (Math.PI * 2)), 65536);
}

// Householder QR solves the same three-column least squares as numpy.linalg.lstsq.
function fit(xs, ys, target) {
  const a = xs.map((x, i) => [x, ys[i], 1]);
  const b = [...target], count = a.length;
  for (let col = 0; col < 3; col++) {
    const vector = a.slice(col).map(row => row[col]);
    const norm = Math.hypot(...vector);
    if (norm < 1e-12) throw new Error('Calibration anchors are singular');
    vector[0] += vector[0] >= 0 ? norm : -norm;
    const weight = 2 / vector.reduce((sum, value) => sum + value * value, 0);
    for (let j = col; j < 3; j++) {
      let dot = 0;
      for (let i = col; i < count; i++) dot += vector[i - col] * a[i][j];
      for (let i = col; i < count; i++) a[i][j] -= weight * dot * vector[i - col];
    }
    let dot = 0;
    for (let i = col; i < count; i++) dot += vector[i - col] * b[i];
    for (let i = col; i < count; i++) b[i] -= weight * dot * vector[i - col];
  }
  const result = [0, 0, 0];
  for (let i = 2; i >= 0; i--) {
    let value = b[i];
    for (let j = i + 1; j < 3; j++) value -= a[i][j] * result[j];
    result[i] = value / a[i][i];
  }
  return result.map(value => round(value * SCALE));
}

function buildLut(indices) {
  const diffs = indices.map((value, i) => mod(indices[(i + 1) % 36] - value, 256));
  if (diffs.some(value => value === 0) || diffs.reduce((a, b) => a + b, 0) !== 256) {
    throw new Error('Pseudo-angle ordering is not strictly circular');
  }
  const x = [indices[0]];
  for (let i = 1; i <= 36; i++) x.push(x[i - 1] + diffs[i - 1]);
  return Array.from({length: 256}, (_, index) => {
    const sample = index < x[0] ? index + 256 : index;
    let interval = 0;
    while (interval < 35 && sample >= x[interval + 1]) interval++;
    const lower = interval * (65536 / 36), upper = (interval + 1) * (65536 / 36);
    const slope = (upper - lower) / (x[interval + 1] - x[interval]);
    return mod(round(lower + slope * (sample - x[interval])), 65536);
  });
}

export function buildBlob(solved) {
  const {affine_q20: affine, angle_lut: lut, commutation_alignment_turn16: alignment,
    forward_angle_sign: sign} = solved;
  if (affine.length !== 6 || lut.length !== 256) throw new Error('Invalid calibration dimensions');
  if (affine.some(value => !Number.isInteger(value) || value < -2147483648 || value > 2147483647)) {
    throw new Error('Affine coefficient out of range');
  }
  if (lut.some(value => !Number.isInteger(value) || value < 0 || value > 65535)
    || !Number.isInteger(alignment) || alignment < 0 || alignment > 65535
    || ![-1, 1].includes(sign)) throw new Error('Invalid calibration value');
  const blob = new Uint8Array(CAL_BLOB_SIZE), data = view(blob);
  data.setUint32(0, 0x314c4143, true);
  data.setUint16(4, 3, true);
  data.setUint16(6, CAL_BLOB_SIZE, true);
  affine.forEach((value, i) => data.setInt32(12 + i * 4, value, true));
  lut.forEach((value, i) => data.setUint16(36 + i * 2, value, true));
  data.setUint16(548, Math.max(0, Math.min(65535, round(solved.fit_max_error_deg * 100))), true);
  data.setUint16(550, Math.max(0, Math.min(65535, round(solved.sweep_delta_deg * 100))), true);
  data.setUint16(552, alignment, true);
  data.setInt8(554, sign);
  data.setUint32(8, crc32(blob.subarray(12)), true);
  return blob;
}

export function parseBlob(blob) {
  if (blob.length !== CAL_BLOB_SIZE) throw new Error('Invalid calibration blob size');
  const data = view(blob);
  if (data.getUint32(0, true) !== 0x314c4143 || data.getUint16(4, true) !== 3
    || data.getUint16(6, true) !== CAL_BLOB_SIZE || ![-1, 1].includes(data.getInt8(554))) {
    throw new Error('Invalid calibration blob format');
  }
  if (data.getUint32(8, true) !== crc32(blob.subarray(12))) throw new Error('Calibration CRC mismatch');
  return {
    affine_q20: Array.from({length: 6}, (_, i) => data.getInt32(12 + i * 4, true)),
    angle_lut: Array.from({length: 256}, (_, i) => data.getUint16(36 + i * 2, true)),
    fit_max_error_deg: data.getUint16(548, true) / 100,
    sweep_delta_deg: data.getUint16(550, true) / 100,
    commutation_alignment_turn16: data.getUint16(552, true),
    forward_angle_sign: data.getInt8(554), blob: blob.slice(), blob_crc32: data.getUint32(8, true),
  };
}

function orderedPoints(points, count) {
  if (points.length !== count) throw new Error(`Capture did not return all ${count} points`);
  const ordered = [...points].sort((a, b) => a.index - b.index);
  if (ordered.some((point, i) => point.index !== i)) throw new Error('Capture indices are incomplete');
  return ordered;
}

export function reduceCapturePoints(points) {
  const ordered = orderedPoints(points, 216);
  const data = Array.from({length: 2}, () => Array.from({length: 36}, () => []));
  for (const point of ordered) {
    if (point.sample_spread > 10000) {
      throw new Error(`Capture sample spread too high at index ${point.index} step ${point.step_index}: ${point.sample_spread}`);
    }
    if (!Number.isInteger(point.step_index) || point.step_index < 0 || point.step_index >= 36) {
      throw new Error('Step index out of range');
    }
    const direction = point.index < 108 ? 0 : 1;
    if (point.sweep_dir !== direction) throw new Error('Capture direction mismatch');
    const rotation = Math.floor((point.index % 108) / 36);
    if (data[direction][point.step_index][rotation]) throw new Error('Duplicate sweep sample');
    data[direction][point.step_index][rotation] = point;
  }
  const reduced = [];
  for (let direction = 0; direction < 2; direction++) {
    for (let local = 0; local < 36; local++) {
      const step = direction ? 35 - local : local, samples = data[direction][step];
      if (samples.length !== 3 || !samples[0] || !samples[1] || !samples[2]) throw new Error('Missing sweep samples');
      const point = {index: reduced.length, step_index: step, sweep_dir: direction};
      for (const key of ['x', 'y', 'z', 'xy_radius', 'sample_spread', 'duty']) {
        point[key] = round(samples.reduce((sum, sample) => sum + sample[key], 0) / 3);
      }
      reduced.push(point);
    }
  }
  return reduced;
}

export function solveCapture(points) {
  const ordered = orderedPoints(points, 72);
  const data = [new Array(36), new Array(36)];
  for (const point of ordered) {
    if (!Number.isInteger(point.step_index) || point.step_index < 0 || point.step_index >= 36) {
      throw new Error('Step index out of range');
    }
    if (![0, 1].includes(point.sweep_dir)) throw new Error('Invalid sweep direction');
    if (data[point.sweep_dir][point.step_index]) throw new Error('Duplicate sweep anchor');
    data[point.sweep_dir][point.step_index] = point;
  }
  const [forward, reverse] = data;
  for (let step = 0; step < 36; step++) {
    if (!forward[step] || !reverse[step]) throw new Error('Missing forward or reverse anchors');
  }
  const xs = forward.map((point, i) => Math.floor((point.x + reverse[i].x) / 2));
  const ys = forward.map((point, i) => Math.floor((point.y + reverse[i].y) / 2));
  const angles = xs.map((_, i) => i * (Math.PI * 2 / 36));
  const affine = [...fit(xs, ys, angles.map(angle => Math.cos(angle) * 16384)),
    ...fit(xs, ys, angles.map(angle => Math.sin(angle) * 16384))];
  const corrected = xs.map((x, i) => applyAffine(x, ys[i], affine));
  const lut = buildLut(corrected.map(([u, v]) => pseudoLookup(u, v)[0]));
  const fitError = Math.max(...corrected.map(([u, v], i) => Math.abs(signedTurn16ErrorDeg(
    round(mod(Math.atan2(v, u), Math.PI * 2) * (65536 / (Math.PI * 2))), i * (65536 / 36)))));
  const sweepDelta = Math.max(...forward.map((point, i) => Math.abs(signedTurn16ErrorDeg(
    angleFromXY(point.x, point.y, affine, lut), angleFromXY(reverse[i].x, reverse[i].y, affine, lut)))));
  if (fitError > 8) throw new Error(`Fit residual too high: ${fitError.toFixed(2)} deg`);
  if (sweepDelta > 12.5) throw new Error(`Sweep disagreement too high: ${sweepDelta.toFixed(2)} deg`);
  const alignment = circularMeanTurn16(ordered.map(point => {
    const angle = calibratedAngleTurn16(point.x, point.y, affine, lut);
    const target = round(((point.step_index % 6) + 0.5) * 65536 / 6);
    return mod(target - angle * 6, 65536);
  }));
  const solved = {affine_q20: affine, angle_lut: lut, fit_max_error_deg: fitError,
    sweep_delta_deg: sweepDelta, commutation_alignment_turn16: alignment, forward_angle_sign: 1};
  solved.blob = buildBlob(solved);
  solved.blob_crc32 = view(solved.blob).getUint32(8, true);
  return solved;
}

function response(bytes, length, command) {
  if (bytes.length !== length) throw new Error(`${command}: invalid response length`);
  if (bytes[0] !== 0) throw new Error(`${command} failed with result ${bytes[0]}`);
  return view(bytes);
}

function checkSignal(signal) {
  if (signal?.aborted) throw new DOMException('Calibration cancelled', 'AbortError');
}

export async function captureCalibration(client, {onProgress, onSample, signal} = {}) {
  checkSignal(signal);
  const brake = await client.exchange(9, new Uint8Array(), 500);
  if (brake[0] !== 0) throw new Error(`Brake failed with result ${brake[0]}`);
  const points = [];
  for (let sweep = 0; sweep < 6; sweep++) {
    checkSignal(signal);
    const direction = sweep < 3 ? 0 : 1, base = sweep * 36;
    const start = await client.exchange(CMD_CAL, Uint8Array.of(CAL_START, direction), 500);
    response(start, 2, 'CAL_START');
    if (start[1] !== 36) throw new Error(`Unexpected point count ${start[1]}`);
    let lastSample = -1;
    const deadline = Date.now() + 120000;
    while (true) {
      const status = await client.exchange(CMD_CAL, Uint8Array.of(CAL_STATUS), 500);
      response(status, 4, 'CAL_STATUS');
      const active = status[1], next = status[2];
      if (next > 36 || status[3] !== 36) throw new Error('Invalid calibration progress');
      onProgress?.({stage: 'capture', done: base + next, total: 216, active: !!active,
        sweep_dir: direction, rotation: sweep % 3 + 1, step_index: Math.min(next, 35)});
      if (active && next > 0 && onSample && next !== lastSample) {
        const live = await client.status();
        const local = next - 1;
        onSample({...live, index: base + local, step_index: direction ? 35 - local : local,
          sweep_dir: direction});
        lastSample = next;
      }
      if (!active) {
        if (next !== 36) throw new Error(`Calibration stopped after ${next} points`);
        break;
      }
      // The firmware completes the active sweep before cancellation is safe.
      if (Date.now() >= deadline) throw new Error('Calibration sweep timed out');
      await wait(50);
    }
    checkSignal(signal);
    for (let local = 0; local < 36; local++) {
      const bytes = await client.exchange(CMD_CAL, Uint8Array.of(CAL_READ_POINT, local), 2000);
      const data = response(bytes, 16, `CAL_READ_POINT ${local}`);
      const step = direction ? 35 - local : local;
      if (bytes[1] !== local || bytes[2] !== step || bytes[3] !== direction) throw new Error('Capture point mismatch');
      const point = {index: base + local, step_index: bytes[2], sweep_dir: bytes[3],
        x: data.getInt16(4, true), y: data.getInt16(6, true), z: data.getInt16(8, true),
        xy_radius: data.getUint16(10, true), sample_spread: data.getUint16(12, true), duty: data.getInt16(14, true)};
      points.push(point);
      onSample?.(point);
    }
  }
  return points;
}

export async function readCalibration(client) {
  const bytes = await client.exchange(CMD_CAL, Uint8Array.of(CAL_INFO), 500);
  const data = response(bytes, 10, 'CAL_INFO');
  const info = {result: bytes[0], valid: bytes[1], blob_size: data.getUint16(2, true),
    blob_crc32: data.getUint32(4, true), commutation_alignment_turn16: data.getUint16(8, true)};
  if (!info.valid) return info;
  if (info.blob_size !== CAL_BLOB_SIZE) throw new Error('Invalid calibration blob size');
  const blob = new Uint8Array(CAL_BLOB_SIZE);
  for (let offset = 0; offset < blob.length;) {
    const count = Math.min(63, blob.length - offset);
    const payload = Uint8Array.of(CAL_READ_BLOB, offset & 255, offset >> 8, count);
    const chunk = await client.exchange(CMD_CAL, payload, 500);
    response(chunk, count + 1, 'CAL_READ_BLOB');
    blob.set(chunk.subarray(1), offset);
    offset += count;
  }
  const solved = parseBlob(blob);
  if (solved.blob_crc32 !== info.blob_crc32
    || solved.commutation_alignment_turn16 !== info.commutation_alignment_turn16) throw new Error('Calibration info mismatch');
  return {...solved, ...info};
}

export async function uploadCalibration(client, solved, {onProgress} = {}) {
  const blob = solved.blob;
  parseBlob(blob);
  onProgress?.({stage: 'upload', done: 0, total: blob.length});
  for (let offset = 0; offset < blob.length; offset += 48) {
    const chunk = blob.subarray(offset, offset + 48);
    const payload = new Uint8Array(3 + chunk.length);
    payload.set([CAL_WRITE_BLOB, offset & 255, offset >> 8]);
    payload.set(chunk, 3);
    const bytes = await client.exchange(CMD_CAL, payload, 500);
    const data = response(bytes, 3, `CAL_WRITE_BLOB ${offset}`);
    const next = data.getUint16(1, true);
    if (next !== offset + chunk.length) throw new Error(`Blob upload stopped at ${next}`);
    onProgress?.({stage: 'upload', done: next, total: blob.length});
  }
  const commit = await client.exchange(CMD_CAL, Uint8Array.of(CAL_COMMIT), 500);
  response(commit, 2, 'CAL_COMMIT');
  if (commit[1] !== 1) throw new Error('Calibration commit was invalid');
  onProgress?.({stage: 'verify', done: 0, total: 1});
  await wait(2000);
  const status = await client.status();
  if (status.result !== 0 || !(status.flags & 1)) throw new Error('ESC came back uncalibrated');
  const stored = await readCalibration(client);
  if (!stored.valid || stored.blob.some((byte, i) => byte !== blob[i])) throw new Error('Calibration readback mismatch');
  onProgress?.({stage: 'verify', done: 1, total: 1});
  return stored;
}

export function captureRows(points, raw = false) {
  return [...points].sort((a, b) => a.index - b.index).map(point => ({...point,
    direction: point.sweep_dir === 0 ? 'forward' : 'reverse',
    rotation: raw ? Math.floor((point.index % 108) / 36) + 1 : ''}));
}

export function calibrationPlots(rawPoints, reducedPoints, solved) {
  const xy = [], field = [], residual = [];
  const colors = ['#27ae60', '#c0392b'];
  const forward = [], reverse = [];
  for (let direction = 0; direction < 2; direction++) {
    const label = direction ? 'reverse' : 'forward';
    const raw = rawPoints.filter(point => point.sweep_dir === direction);
    const reduced = reducedPoints.filter(point => point.sweep_dir === direction).sort((a, b) => a.step_index - b.step_index);
    const fieldPoints = (reduced.length ? reduced : raw).slice().sort((a, b) => a.step_index - b.step_index);
    xy.push({name: `${label} raw`, x: raw.map(p => p.x), y: raw.map(p => p.y), color: colors[direction],
      pointsOnly: true, marker: 'circle', size: 4, pointColor: colors[direction] + '66'});
    xy.push({name: `${label} reduced`, x: reduced.map(p => p.x), y: reduced.map(p => p.y), color: colors[direction],
      marker: 'cross', size: 9});
    for (const axis of ['x', 'y']) field.push({name: `${label} ${axis.toUpperCase()}`,
      x: fieldPoints.map(p => p.step_index), y: fieldPoints.map(p => p[axis]),
      color: axis === 'x' ? colors[direction] : (direction ? '#8e44ad' : '#2e86ab'),
      marker: axis === 'x' ? 'circle' : 'triangle', size: 4});
    if (solved && reduced.length === 36) {
      const errors = reduced.map(p => signedTurn16ErrorDeg(
        angleFromXY(p.x, p.y, solved.affine_q20, solved.angle_lut), p.step_index * (65536 / 36)));
      (direction ? reverse : forward).push(...errors);
      residual.push({name: label, x: reduced.map(p => p.step_index), y: errors, color: colors[direction]});
    }
  }
  if (forward.length === 36 && reverse.length === 36) {
    const steps = Array.from({length: 36}, (_, i) => i);
    residual.push({name: 'even', x: steps, y: forward.map((f, i) => (f + reverse[i]) / 2), color: '#2980b9'},
      {name: 'hysteresis', x: steps, y: forward.map((f, i) => (f - reverse[i]) / 2), color: '#8e44ad'});
  }
  return {xy, field, residual};
}
