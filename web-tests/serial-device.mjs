import { crc8, decodeFrame, encodeFrame, pack } from '../docs/protocol.js';
import { PACKET, cobsDecode, cobsEncode, encodePacket } from '../docs/serial.js';

export function statusPayload(position = 65536) {
  return pack('BBBBhhhHiihHHHHHHIIIIIIH', 0, 1, 17, 0, 123, -456, 789, 321, position, -32768, 50, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 250);
}

export class FakePort {
  constructor(onWrite, { fragment = 0, delay = 0 } = {}) {
    this.onWrite = onWrite;
    this.fragment = fragment;
    this.delay = delay;
    this.openCount = 0;
    this.closeCount = 0;
    this.writes = [];
    this.readable = null;
    this.writable = null;
  }
  async open() {
    if (this.readable) throw new Error('Port already open');
    this.openCount++;
    this.readable = new ReadableStream({ start: controller => { this.controller = controller; }, cancel: () => { this.controller = null; } });
    this.writable = new WritableStream({ write: async data => {
      this.writes.push({ data: data.slice(), time: performance.now() });
      const response = await this.onWrite(data);
      if (response?.length) {
        if (this.delay) setTimeout(() => this.feed(response), this.delay);
        else this.feed(response);
      }
    } });
  }
  feed(data) {
    if (!this.controller) return;
    const size = this.fragment || data.length;
    for (let start = 0; start < data.length; start += size) this.controller.enqueue(data.slice(start, start + size));
  }
  async setSignals() {}
  getInfo() { return { usbVendorId: 0x303a, usbProductId: 0x1001 }; }
  async close() {
    if (this.readable?.locked || this.writable?.locked) throw new Error('Port streams still locked');
    this.controller = null;
    this.readable = null; this.writable = null; this.closeCount++;
  }
}

export function appResponse(data) {
  const frame = decodeFrame(data);
  return encodeFrame(frame.address, frame.command, frame.command === 14 ? pack('ii', 65536, -32768) : statusPayload());
}

export class RomDevice {
  constructor() {
    this.memory = new Uint8Array(16384).fill(0x5a);
    this.state = 'command';
    this.address = 0;
    this.erased = [];
    this.written = [];
    this.reads = [];
    this.running = false;
    this.corruptRead = false;
  }
  write(data) {
    const ack = new Uint8Array([0x79]);
    if (this.state === 'command') {
      if (data.length === 1 && data[0] === 0x7f) return ack;
      if (data.length !== 2 || (data[0] ^ data[1]) !== 255) throw new Error('Bad command checksum');
      this.command = data[0];
      this.state = this.command === 0x44 ? 'erase' : 'address';
      return ack;
    }
    if (this.state === 'address') {
      if (data.length !== 5 || data.reduce((a, b) => a ^ b, 0)) throw new Error('Bad address checksum');
      this.address = new DataView(data.buffer, data.byteOffset).getUint32(0, false);
      this.state = this.command === 0x21 ? 'command' : this.command === 0x31 ? 'write' : 'read';
      if (this.command === 0x21) this.running = true;
      return ack;
    }
    if (this.state === 'erase') {
      if (data.reduce((a, b) => a ^ b, 0)) throw new Error('Bad erase checksum');
      const view = new DataView(data.buffer, data.byteOffset);
      const count = view.getUint16(0, false) + 1;
      if (data.length !== 3 + count * 2) throw new Error('Bad erase count');
      for (let i = 0; i < count; i++) {
        const page = view.getUint16(2 + i * 2, false);
        this.erased.push(page); this.memory.fill(255, 0x580 + page * 128, 0x580 + (page + 1) * 128);
      }
      this.state = 'command'; return ack;
    }
    const offset = this.address - 0x08000000;
    if (this.state === 'write') {
      if (data.length !== data[0] + 3 || data.reduce((a, b) => a ^ b, 0)) throw new Error('Bad write checksum');
      this.memory.set(data.subarray(1, -1), offset); this.written.push(this.address);
      this.state = 'command'; return ack;
    }
    if (this.state === 'read') {
      if (data.length !== 2 || (data[0] ^ data[1]) !== 255) throw new Error('Bad read count');
      const length = data[0] + 1;
      this.reads.push(this.address);
      const reply = new Uint8Array(1 + length); reply[0] = 0x79; reply.set(this.memory.subarray(offset, offset + length), 1);
      if (this.corruptRead) reply[1] ^= 1;
      this.state = 'command'; return reply;
    }
    throw new Error('Unknown bootloader state');
  }
}

// Test model of firmware/Lib/pennyesc_bridge.h. `uart(bytes, baud)` is the ESC
// side: it returns reply bytes (or a promise of them) or null for silence.
export class FakeBridge {
  constructor(uart, { fragment = 0, delay = 0, board = 'qtpy-esp32s3', pins = [12, 13, -1], version = 2, ready = 0 } = {}) {
    Object.assign(this, { uart, board, pins, version });
    this.aux = { pin: 11, pulse: 1500, enabled: false };
    this.readyAt = performance.now() + ready;
    this.uartWrites = [];
    this.commands = [];
    this.stream = null;
    this.pinsSaved = 0;
    this.buffer = [];
    this.port = new FakePort(data => this.receive(data), { fragment, delay });
  }

  async receive(data) {
    for (const byte of data) {
      if (byte !== 0) { this.buffer.push(byte); continue; }
      const packet = this.buffer.length ? cobsDecode(this.buffer) : null;
      this.buffer = [];
      if (packet && packet.length >= 2 && crc8(packet.subarray(0, -1)) === packet.at(-1)) await this.handle(packet);
    }
    return null;
  }

  send(type, payload) {
    if (!this.port.controller) { this.stopStream(); return; }
    const encoded = cobsEncode(encodePacket(type, payload));
    const framed = new Uint8Array(encoded.length + 2); framed.set(encoded, 1);
    this.port.feed(framed);
  }

  state(status = 0) {
    const name = new TextEncoder().encode(this.board);
    this.send(PACKET.STATE, [status, this.version, this.pins[0] & 255, this.pins[1] & 255, this.pins[2] & 255,
      this.aux.pin & 255, this.aux.pulse & 255, this.aux.pulse >> 8, this.aux.enabled ? 1 : 0, ...name]);
  }

  async handle(packet) {
    const type = packet[0], data = packet.subarray(1, -1);
    const signed = value => (value << 24) >> 24;
    if (type !== PACKET.UART) this.commands.push(type);
    if (performance.now() < this.readyAt) return;
    if (type === PACKET.HELLO) { this.stopStream(); this.state(); }
    else if (type === PACKET.PINS) {
      const pins = [...data].map(signed);
      if (pins[0] === pins[1] || pins.slice(0, 2).includes(pins[2]) && pins[2] >= 0) return this.state(1);
      this.pins = pins; this.pinsSaved++; this.state();
    } else if (type === PACKET.AUX) {
      const pin = signed(data[0]), pulse = data[1] | data[2] << 8;
      if (pulse === 0) this.aux.enabled = false;
      else if (this.pins.includes(pin) || pulse < 1000 || pulse > 2000) return this.state(1);
      else this.aux = { pin, pulse, enabled: true };
      this.state();
    } else if (type === PACKET.STREAM) {
      this.stopStream();
      if (data[1]) this.stream = setInterval(() => void this.poll(data[0]), 1000 / data[1]);
      this.state();
    } else if (type === PACKET.UART) {
      const baud = data[0], bytes = data.slice(1);
      this.uartWrites.push({ baud, bytes, time: performance.now() });
      // Like the real UART, the reply arrives independently of the write.
      void Promise.resolve(this.uart(bytes, baud)).then(reply => { if (reply?.length) this.rx(reply, baud); });
    } else this.state(4);
  }

  rx(bytes, baud) {
    for (let start = 0; start < bytes.length; start += 100) this.send(PACKET.UART_RX, [baud, ...bytes.subarray(start, start + 100)]);
  }

  async poll(address) {
    const reply = await this.uart(encodeFrame(address, 1), 0);
    if (!reply?.length) return;
    const stamp = new Uint8Array(4);
    new DataView(stamp.buffer).setUint32(0, Math.round(performance.now() * 1000) >>> 0, true);
    this.send(PACKET.TELEMETRY, [...stamp, ...reply]);
  }

  stopStream() { clearInterval(this.stream); this.stream = null; }
}
