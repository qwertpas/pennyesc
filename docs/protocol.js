export const CMD = Object.freeze({
  GET_STATUS: 1, SET_POSITION: 2, SET_DUTY: 3, CAL: 4, DEBUG: 5,
  ZERO_POSITION: 6, SET_VELOCITY: 7, SET_CONTROL: 8, BRAKE: 9,
  SEND_POSITION: 10, ENTER_BOOT: 11, SET_ADVANCE: 12, SET_QUIET: 13,
  GET_POS_VEL: 14, SEND_DUTY: 15,
});
export const CAL = Object.freeze({ START: 1, STATUS: 2, READ_POINT: 3, WRITE_BLOB: 4, COMMIT: 5, CLEAR: 6, INFO: 7, READ_BLOB: 8 });
export const DEBUG = Object.freeze({ CAPTURE_START: 1, CAPTURE_STATUS: 2, CAPTURE_READ: 3, SET_OBSERVER: 4, STEP_SET: 5, STEP_TRANSITION: 6 });
export const BOOT_MAGIC = 0x424f4f54;
export const DUTY_LIMIT = 799;
export const TURN_TO_RAD = 2 * Math.PI / 65536;
const RAD_TO_TURN = 65536 / (2 * Math.PI);
export function checkDuty(value) {
  if (!Number.isInteger(value) || value < -DUTY_LIMIT || value > DUTY_LIMIT) {
    throw new Error(`Duty must be an integer from -${DUTY_LIMIT} to ${DUTY_LIMIT}`);
  }
  return value;
}
export function checkClip(value) {
  if (!Number.isInteger(value) || value < 0 || value > DUTY_LIMIT) {
    throw new Error(`Clip must be an integer from 0 to ${DUTY_LIMIT}`);
  }
  return value;
}
export function round(value) {
  const low = Math.floor(value);
  return value - low === 0.5 ? low + Math.abs(low % 2) : Math.floor(value + 0.5);
}

export function crc8(data) {
  let crc = 0;
  for (const value of data) {
    crc ^= value;
    for (let bit = 0; bit < 8; bit++) crc = ((crc << 1) ^ (crc & 128 ? 7 : 0)) & 255;
  }
  return crc;
}

export function encodeFrame(address, command, payload = new Uint8Array()) {
  if (!Number.isInteger(address) || address < 0 || address > 15) throw new Error('Address must be 0–15');
  if (!Number.isInteger(command) || command < 0 || command > 15) throw new Error('Command must be 0–15');
  if (payload.length > 64) throw new Error('Payload exceeds 64 bytes');
  const frame = new Uint8Array(payload.length + 4);
  frame.set([0xaa, (address << 4) | command, payload.length]);
  frame.set(payload, 3);
  frame[frame.length - 1] = crc8(frame.subarray(0, -1));
  return frame;
}

export function decodeFrame(frame) {
  if (frame.length < 4 || frame[0] !== 0xaa || frame[2] > 64 || frame.length !== frame[2] + 4) throw new Error('Invalid frame');
  if (crc8(frame.subarray(0, -1)) !== frame[frame.length - 1]) throw new Error('Frame CRC mismatch');
  return { address: frame[1] >> 4, command: frame[1] & 15, payload: frame.slice(3, -1) };
}

export class FrameReader {
  constructor() { this.buffer = new Uint8Array(); this.frames = []; this.crcErrors = 0; }
  feed(chunk) {
    const data = new Uint8Array(this.buffer.length + chunk.length);
    data.set(this.buffer); data.set(chunk, this.buffer.length);
    let offset = 0;
    while (offset < data.length) {
      if (data[offset] !== 0xaa) { offset++; continue; }
      if (data.length - offset < 3) break;
      const size = data[offset + 2] + 4;
      if (size > 68) { offset++; continue; }
      if (data.length - offset < size) break;
      try { this.frames.push(decodeFrame(data.subarray(offset, offset + size))); offset += size; }
      catch { this.crcErrors++; offset++; }
    }
    this.buffer = data.slice(offset);
  }
  take(address, command) {
    const index = this.frames.findIndex(frame => frame.address === address && frame.command === command);
    return index < 0 ? null : this.frames.splice(index, 1)[0].payload;
  }
}

const TYPES = { B: [1, 'Uint8'], b: [1, 'Int8'], H: [2, 'Uint16'], h: [2, 'Int16'], I: [4, 'Uint32'], i: [4, 'Int32'], f: [4, 'Float32'] };
function fields(format) {
  if (format[0] === '<') format = format.slice(1);
  return [...format].map(type => { if (!TYPES[type]) throw new Error(`Unknown field ${type}`); return TYPES[type]; });
}
export function pack(format, ...values) {
  const types = fields(format);
  if (types.length !== values.length) throw new Error('Wrong field count');
  const data = new Uint8Array(types.reduce((n, [size]) => n + size, 0));
  const view = new DataView(data.buffer);
  let offset = 0;
  types.forEach(([size, type], index) => {
    const value = values[index];
    if (!Number.isFinite(value)) throw new Error('Value must be finite');
    if (type !== 'Float32') {
      const signed = type.startsWith('Int');
      const min = signed ? -(2 ** (size * 8 - 1)) : 0;
      const max = 2 ** (size * 8 - (signed ? 1 : 0)) - 1;
      if (!Number.isInteger(value) || value < min || value > max) throw new Error(`Value outside ${type} range`);
    }
    view[`set${type}`](offset, value, true); offset += size;
  });
  return data;
}
export function unpack(format, data) {
  const types = fields(format);
  if (data.length !== types.reduce((n, [size]) => n + size, 0)) throw new Error('Wrong response length');
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  let offset = 0;
  return types.map(([size, type]) => { const value = view[`get${type}`](offset, true); offset += size; return value; });
}

const STATUS_NAMES = ['result', 'mode', 'flags', 'faults', 'x', 'y', 'z', 'angle_turn16', 'position_turn32', 'velocity_turn32_per_s', 'duty', 'mct_fault_count', 'isr_us', 'isr_max_us', 'i2c_us', 'i2c_start_us', 'i2c_end_us', 'isr_overrun_count', 'i2c_timeout_count', 'i2c_nack_count', 'i2c_recover_count', 'uart_overrun_errors', 'tmag_sample_count', 'tmag_sample_dt_us'];
function motion(values) {
  return Object.assign(values, {
    position_rad: values.position_turn32 * TURN_TO_RAD,
    velocity_rad_s: values.velocity_turn32_per_s * TURN_TO_RAD,
    velocity_rpm: values.velocity_turn32_per_s * 60 / 65536,
  });
}
export function unpackStatus(data) {
  const format = data.length === 60 ? 'BBBBhhhHiihHHHHHHIIIIIIH' : 'BBBBhhhHiihHHHHHHIIIII';
  const values = unpack(format, data);
  const status = Object.fromEntries(STATUS_NAMES.map((name, i) => [name, values[i] ?? 0]));
  status.calibrated = Boolean(status.flags & 1);
  return motion(status);
}
const RESULTS = ['OK', 'Bad state', 'Bad argument', 'Not calibrated', 'Busy', 'Out of range', 'Flash failed', 'CRC mismatch'];
export function checkResult(result) { if (result !== 0) throw new Error(RESULTS[result] || `Device result ${result}`); }

export class PennyClient {
  constructor(connection, address = 0) { this.connection = connection; this.address = address; }
  exchange(command, payload = new Uint8Array(), timeout = 500) { return this.connection.exchange(this.address, command, payload, timeout); }
  async status(timeout = 500) { return unpackStatus(await this.exchange(CMD.GET_STATUS, new Uint8Array(), timeout)); }
  async posVel(timeout = 200) {
    const [position_turn32, velocity_turn32_per_s] = unpack('ii', await this.exchange(CMD.GET_POS_VEL, new Uint8Array(), timeout));
    return motion({ position_turn32, velocity_turn32_per_s });
  }
  async command(command, payload = new Uint8Array(), timeout = 500) {
    const status = unpackStatus(await this.exchange(command, payload, timeout)); checkResult(status.result); return status;
  }
  setDuty(duty) { return this.command(CMD.SET_DUTY, pack('h', checkDuty(duty))); }
  brake() { return this.command(CMD.BRAKE); }
  setAdvance(degrees) { return this.command(CMD.SET_ADVANCE, pack('h', degrees)); }
  setPosition(position) { return this.command(CMD.SET_POSITION, pack('i', position)); }
  setPositionRad(radians) { return this.setPosition(round(radians * RAD_TO_TURN)); }
  setVelocity(velocity) { return this.command(CMD.SET_VELOCITY, pack('i', velocity)); }
  setVelocityRad(velocity) { return this.setVelocity(round(velocity * RAD_TO_TURN)); }
  setVelocityRpm(rpm) { return this.setVelocity(round(rpm * 65536 / 60)); }
  zero() { return this.command(CMD.ZERO_POSITION); }
  setControl({ kp = 0, kd = 0, kv = 0, kf = 0, clip = 150, deadband_turn32, brushed = false }) {
    checkDuty(kf); checkClip(clip);
    const values = [round(kp * 256), round(kd * 256), round(kv * 256), kf, clip];
    let payload = pack(brushed ? 'iihhh' : 'hhhhh', ...values);
    if (!brushed && deadband_turn32 !== undefined) payload = new Uint8Array([...payload, ...pack('H', deadband_turn32)]);
    return this.command(CMD.SET_CONTROL, payload);
  }
  sendPosition(position) { return this.connection.send(this.address, CMD.SEND_POSITION, pack('i', position)); }
  sendTrajectory(positionRad, velocityRad, feedforward) {
    checkDuty(feedforward);
    return this.connection.send(this.address, CMD.SEND_POSITION, pack('iih', round(positionRad * RAD_TO_TURN), round(velocityRad * RAD_TO_TURN), feedforward));
  }
  sendDuty(duty) { return this.connection.send(this.address, CMD.SEND_DUTY, pack('h', checkDuty(duty))); }
  async calStart(direction) { const values = unpack('BB', await this.exchange(CMD.CAL, pack('BB', CAL.START, direction))); checkResult(values[0]); return values; }
  async calStatus() {
    const [result, active, next_index, total_points] = unpack('BBBB', await this.exchange(CMD.CAL, pack('B', CAL.STATUS)));
    checkResult(result); return { result, active, next_index, total_points };
  }
  async calReadPoint(index) {
    const values = unpack('BBBBhhhHHh', await this.exchange(CMD.CAL, pack('BB', CAL.READ_POINT, index), 2000));
    checkResult(values[0]);
    return Object.fromEntries(['result', 'index', 'step_index', 'sweep_dir', 'x', 'y', 'z', 'xy_radius', 'sample_spread', 'duty'].map((key, i) => [key, values[i]]));
  }
  async calWriteBlob(offset, chunk) {
    const values = unpack('BH', await this.exchange(CMD.CAL, new Uint8Array([...pack('BH', CAL.WRITE_BLOB, offset), ...chunk])));
    checkResult(values[0]); return values;
  }
  async calCommit() { const values = unpack('BB', await this.exchange(CMD.CAL, pack('B', CAL.COMMIT))); checkResult(values[0]); return values; }
  async calClear() { const values = unpack('BB', await this.exchange(CMD.CAL, pack('B', CAL.CLEAR))); checkResult(values[0]); return values; }
  async calInfo() {
    const values = unpack('BBHIH', await this.exchange(CMD.CAL, pack('B', CAL.INFO)));
    checkResult(values[0]);
    return Object.fromEntries(['result', 'valid', 'blob_size', 'blob_crc32', 'commutation_alignment_turn16'].map((key, i) => [key, values[i]]));
  }
  async calReadBlob() {
    const blob = new Uint8Array(640);
    for (let offset = 0; offset < blob.length; offset += 63) {
      const count = Math.min(63, blob.length - offset);
      const reply = await this.exchange(CMD.CAL, pack('BHB', CAL.READ_BLOB, offset, count));
      if (reply.length !== count + 1) throw new Error('Invalid calibration response');
      checkResult(reply[0]); blob.set(reply.subarray(1), offset);
    }
    return blob;
  }
}
