// Test-only hardware model. The browser harness loads the unchanged production app.
import {decodeFrame, encodeFrame, pack, unpack} from '../docs/protocol.js';
import {parseBlob} from '../docs/calibration.js';
import {FakePort} from './serial-device.mjs';

const encoder = new TextEncoder(), decoder = new TextDecoder();

export class BrowserDevice {
  constructor(fixture) {
    const entry = fixture.cases[0];
    this.raw = entry.raw.map(row => Object.fromEntries(fixture.fields.map((key, i) => [key, row[i]])));
    this.blob = Uint8Array.from(entry.solved.blob.match(/../g), pair => parseInt(pair, 16));
    this.board = 'xiao-esp32s3';
    this.pins = [7, 44, -1];
    this.mode = 'off';
    this.address = 1;
    this.sample = 0;
    this.started = performance.now();
    this.duty = 0;
    this.velocity = 0;
    this.position = 0;
    this.control = [0, 0, 0, 0, 150];
    this.sweep = -1;
    this.next = 0;
    this.active = false;
    this.upload = new Uint8Array(640);
    this.offset = 0;
    this.stats = {statusReplies: 0, sweeps: 0, pointsRead: 0, bytesUploaded: 0, commits: 0, pinsSaved: 0};
  }

  status(result = 0) {
    const seconds = (performance.now() - this.started) / 1000;
    const phase = seconds * 2.4;
    const point = this.active ? this.raw[this.sweep * 36 + Math.max(0, this.next - 1)] : {
      x: Math.round(14500 * Math.cos(phase) + 1900 * Math.sin(phase) + 700),
      y: Math.round(-2600 * Math.cos(phase) + 24000 * Math.sin(phase) - 1200),
      z: Math.round(2300 + Math.sin(phase * 0.7) * 700),
    };
    this.sample++;
    this.stats.statusReplies++;
    const position = this.position + Math.round(seconds * (this.velocity || 25000));
    return pack('BBBBhhhHiihHHHHHHIIIIIIH', result, this.active ? 2 : 1,
      16 | (this.blob ? 1 : 0), 0, point.x, point.y, point.z,
      ((position % 65536) + 65536) % 65536, position, this.velocity || 25000,
      this.duty, 0, 38, 45, 12, 3, 9, 0, 0, 0, 0, 0, this.sample, 1000);
  }

  pinsText() { return `# pins board=${this.board} rx=${this.pins[0]} tx=${this.pins[1]} gnd=${this.pins[2]}\n`; }

  shell(bytes) {
    const command = decoder.decode(bytes).trim().replace(/^!#/, '');
    if (command === 'ping') return encoder.encode('pong\n');
    if (command === 'pins') return encoder.encode(this.pinsText());
    if (command.startsWith('pins ')) {
      const pins = command.slice(5).split(/\s+/).map(Number);
      if (pins.length !== 3 || pins.some(pin => !Number.isInteger(pin))) return encoder.encode('err pins\n');
      this.pins = pins;
      this.stats.pinsSaved++;
      return encoder.encode(this.pinsText());
    }
    if (command.startsWith('bridge ')) {
      this.mode = command.slice(7);
      return encoder.encode(`# bridge=${this.mode}\n`);
    }
    return encoder.encode('err command\n');
  }

  calibration(payload) {
    switch (payload[0]) {
      case 1:
        if (this.active) return Uint8Array.of(4, 36);
        this.sweep = (this.sweep + 1) % 6;
        if (payload[1] !== (this.sweep < 3 ? 0 : 1)) return Uint8Array.of(2, 36);
        this.next = 0;
        this.active = true;
        this.stats.sweeps++;
        return Uint8Array.of(0, 36);
      case 2:
        if (this.active) this.next = Math.min(36, this.next + 1);
        this.active = this.next < 36;
        return Uint8Array.of(0, this.active ? 1 : 0, this.next, 36);
      case 3: {
        const local = payload[1], point = this.raw[this.sweep * 36 + local];
        if (!point || this.active) throw new Error('Read calibration point before sweep completed');
        this.stats.pointsRead++;
        return pack('BBBBhhhHHh', 0, local, point.step_index, point.sweep_dir,
          point.x, point.y, point.z, point.xy_radius, point.sample_spread, point.duty);
      }
      case 4: {
        const offset = payload[1] | payload[2] << 8;
        if (offset === 0) { this.offset = 0; this.upload.fill(0); }
        if (offset !== this.offset || offset + payload.length - 3 > 640) return pack('BH', 5, this.offset);
        this.upload.set(payload.subarray(3), offset);
        this.offset += payload.length - 3;
        this.stats.bytesUploaded += payload.length - 3;
        return pack('BH', 0, this.offset);
      }
      case 5:
        if (this.offset !== 640) return Uint8Array.of(7, 0);
        parseBlob(this.upload);
        this.blob = this.upload.slice();
        this.stats.commits++;
        return Uint8Array.of(0, 1);
      case 6: this.blob = null; return Uint8Array.of(0, 0);
      case 7: {
        const solved = this.blob && parseBlob(this.blob);
        return pack('BBHIH', 0, solved ? 1 : 0, solved ? 640 : 0,
          solved?.blob_crc32 ?? 0, solved?.commutation_alignment_turn16 ?? 0);
      }
      case 8: {
        if (!this.blob) return Uint8Array.of(3);
        const offset = payload[1] | payload[2] << 8;
        return new Uint8Array([0, ...this.blob.slice(offset, offset + payload[3])]);
      }
      default: throw new Error('Unknown calibration command');
    }
  }

  write(bytes) {
    if (bytes[0] !== 0xaa) return this.shell(bytes);
    const {address, command, payload} = decodeFrame(bytes);
    if (this.mode !== 'app') throw new Error('Binary request outside bridge app mode');
    if (address !== this.address) return null;
    let response;
    switch (command) {
      case 1: response = this.status(); break;
      case 2: this.position = unpack('i', payload)[0]; response = this.status(); break;
      case 3: this.duty = unpack('h', payload)[0]; this.control = [0, 0, 0, this.duty, this.control[4]]; response = this.status(); break;
      case 4: response = this.calibration(payload); break;
      case 6: this.position = 0; this.started = performance.now(); response = this.status(); break;
      case 7: this.velocity = unpack('i', payload)[0]; response = this.status(); break;
      case 8: this.control = unpack('hhhhh', payload); this.duty = this.control[3]; response = this.status(); break;
      case 9: this.duty = 0; this.control = [0, 0, 0, 0, this.control[4]]; response = this.status(); break;
      case 10: this.position = unpack('i', payload)[0]; return null;
      case 12: unpack('h', payload); response = this.status(); break;
      case 14: response = pack('ii', this.position, this.velocity); break;
      case 15: this.duty = unpack('h', payload)[0]; this.control = [0, 0, 0, this.duty, this.control[4]]; return null;
      default: throw new Error(`Unsupported test device command ${command}`);
    }
    return encodeFrame(address, command, response);
  }
}

export async function installDevice() {
  const response = await fetch(new URL('./calibration-fixture.json', import.meta.url));
  if (!response.ok) throw new Error('Could not load calibration fixture');
  const device = new BrowserDevice(await response.json());
  const port = new FakePort(bytes => device.write(bytes), {fragment: 13, delay: 1});
  const serial = new EventTarget();
  serial.requestPort = async () => port;
  serial.getPorts = async () => [port];
  Object.defineProperty(navigator, 'serial', {value: serial, configurable: true});
  window.pennyTest = {device, port, errors: []};
  window.addEventListener('error', event => window.pennyTest.errors.push(event.message));
  window.addEventListener('unhandledrejection', event => window.pennyTest.errors.push(String(event.reason)));
  return device;
}
