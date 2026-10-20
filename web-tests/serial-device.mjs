import { decodeFrame, encodeFrame, pack } from '../docs/protocol.js';

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
