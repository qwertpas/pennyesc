import { crc8, encodeFrame, FrameReader, decodeFrame, unpackStatus } from './protocol.js';

// Host side of the stateless bridge protocol in firmware/Lib/pennyesc_bridge.h.
// Every message is a packet [type, payload..., crc8]; USB packets are COBS
// encoded between 0x00 delimiters. The bridge keeps no mode: each UART packet
// carries the baud it is sent at.

export class TimeoutError extends Error {
  constructor(message = 'Serial response timed out') { super(message); this.name = 'TimeoutError'; }
}

export const BRIDGE_VERSION = 2;
export const BAUD = Object.freeze({ APP: 0, BOOT: 1 });
export const PACKET = Object.freeze({ HELLO: 1, PINS: 2, AUX: 3, STREAM: 4, UART: 0x10, UART_RX: 0x11, TELEMETRY: 0x12, STATE: 0x81 });
const BRIDGE_ERRORS = ['OK', 'Invalid pin or value', 'Bridge could not save settings', 'Bridge could not start PWM',
  'Bridge firmware does not support this command', 'USB is using the bridge'];

export function cobsEncode(data) {
  const out = [0];
  let code = 1, codeIndex = 0;
  for (const byte of data) {
    if (byte === 0) { out[codeIndex] = code; codeIndex = out.length; out.push(0); code = 1; continue; }
    out.push(byte);
    if (++code === 255) { out[codeIndex] = code; codeIndex = out.length; out.push(0); code = 1; }
  }
  out[codeIndex] = code;
  return Uint8Array.from(out);
}

export function cobsDecode(data) {
  const out = [];
  for (let i = 0; i < data.length;) {
    const code = data[i++];
    if (code === 0 || i + code - 1 > data.length) return null;
    for (let j = 1; j < code; j++) out.push(data[i++]);
    if (code !== 255 && i < data.length) out.push(0);
  }
  return Uint8Array.from(out);
}

export function encodePacket(type, payload = []) {
  const packet = new Uint8Array(payload.length + 2);
  packet[0] = type; packet.set(payload, 1);
  packet[packet.length - 1] = crc8(packet.subarray(0, -1));
  return packet;
}

export function parseState(data) {
  if (data.length < 9) throw new Error('Invalid bridge reply');
  const signed = value => (value << 24) >> 24;
  return {
    status: data[0], version: data[1], rx: signed(data[2]), tx: signed(data[3]), gnd: signed(data[4]),
    aux: { pin: signed(data[5]), pulse: data[6] | data[7] << 8, enabled: data[8] === 1 },
    board: new TextDecoder().decode(data.subarray(9)),
  };
}

export class SerialConnection extends EventTarget {
  static owner = null;

  constructor() {
    super();
    this.port = null;
    this.connected = false;
    this.state = null;
    this.samples = [];
    this._tail = Promise.resolve();
    this._chunks = [];
    this._offset = 0;
    this._waiter = null;
    this._frames = new FrameReader();
    this._baud = BAUD.APP;
    this._reply = null;
  }

  // Transport hooks; BleConnection replaces these three.
  async _open(port) {
    this.port = port || await navigator.serial.requestPort();
    await this.port.open({ baudRate: 115200, bufferSize: 65536 });
    await this.port.setSignals({ dataTerminalReady: false, requestToSend: false });
    this._reader = this.port.readable.getReader();
    this._writer = this.port.writable.getWriter();
    this._pump = this._read();
  }
  async _transmit(packet) {
    const encoded = cobsEncode(packet);
    const data = new Uint8Array(encoded.length + 2);
    data.set(encoded, 1);
    await this._writer.write(data);
  }
  async _shutdown() {
    try {
      await this._reader?.cancel();
      await this._pump;
    } finally {
      this._reader?.releaseLock(); this._writer?.releaseLock();
      this._reader = null; this._writer = null;
      if (this.port?.readable) await this.port.close();
    }
  }

  // Opens the transport and greets the bridge.
  async connect(port) {
    if (this.connected) throw new Error('Already connected');
    if (SerialConnection.owner) throw new Error('Another connection owns the port');
    SerialConnection.owner = this;
    this._failure = null;
    this._closing = false;
    this._baud = BAUD.APP;
    try {
      await this._open(port);
      this.connected = true;
      this.state = await this._hello();
      return this;
    } catch (error) {
      await this.disconnect();
      throw error;
    }
  }

  async _read() {
    let buffer = [];
    try {
      while (true) {
        const { value, done } = await this._reader.read();
        if (done) break;
        for (const byte of value) {
          if (byte !== 0) { if (buffer.length < 600) buffer.push(byte); continue; }
          const packet = buffer.length ? cobsDecode(buffer) : null;
          buffer = [];
          if (packet) this._packet(packet);
        }
      }
      if (!this._closing) throw new Error('Serial port disconnected');
    } catch (error) { this._lost(error); }
  }

  _lost(error) {
    if (this._closing) return;
    this._failure = error;
    this.connected = false;
    this._wake(); this.sampleWake?.();
    this._reply?.reject(error);
    const event = new Event('disconnect'); event.error = error; this.dispatchEvent(event);
    // Close outside the read pump so closing can await the pump safely.
    queueMicrotask(() => { void this.disconnect().catch(() => {}); });
  }

  _packet(packet) {
    if (packet.length < 2 || crc8(packet.subarray(0, -1)) !== packet[packet.length - 1]) return;
    const data = packet.subarray(1, -1);
    if (packet[0] === PACKET.UART_RX) {
      // Bytes received at another baud belong to an earlier operation.
      if (data[0] === this._baud && data.length > 1) { this._chunks.push(data.slice(1)); this._wake(); }
    } else if (packet[0] === PACKET.TELEMETRY) this._telemetry(data);
    else if (packet[0] === PACKET.STATE && this._reply) this._reply.resolve(data.slice());
  }

  disconnect() {
    if (this._closeTask) return this._closeTask;
    this._closeTask = (async () => {
      this._closing = true;
      this.connected = false;
      this._failure ||= new Error('Disconnected');
      this._wake(); this.sampleWake?.();
      this._reply?.reject(this._failure);
      try {
        await this._tail.catch(() => {});
        await this._shutdown();
      } finally {
        this._clear(); this.samples = []; this.streaming = false;
        if (SerialConnection.owner === this) SerialConnection.owner = null;
      }
    })().finally(() => { this._closeTask = null; });
    return this._closeTask;
  }

  _wake() { if (this._waiter) { const wake = this._waiter; this._waiter = null; wake(); } }
  async _wait(deadline) {
    if (this._failure) throw this._failure;
    if (this._chunks.length) return;
    const timeout = deadline - performance.now();
    if (timeout <= 0) throw new TimeoutError();
    await new Promise((resolve, reject) => {
      const timer = setTimeout(() => { this._waiter = null; reject(new TimeoutError()); }, timeout);
      this._waiter = () => { clearTimeout(timer); resolve(); };
    });
    if (this._failure) throw this._failure;
  }
  _take(count = Infinity) {
    const chunk = this._chunks[0];
    if (!chunk) return new Uint8Array();
    const end = Math.min(chunk.length, this._offset + count);
    const data = chunk.subarray(this._offset, end);
    this._offset = end;
    if (end === chunk.length) { this._chunks.shift(); this._offset = 0; }
    return data;
  }
  async _readBytes(count, timeout = 1000) {
    const data = new Uint8Array(count);
    let offset = 0;
    const deadline = performance.now() + timeout;
    while (offset < count) {
      await this._wait(deadline);
      const chunk = this._take(count - offset);
      data.set(chunk, offset); offset += chunk.length;
    }
    return data;
  }
  async _readFrame(address, command, timeout) {
    const deadline = performance.now() + timeout;
    while (true) {
      const payload = this._frames.take(address, command);
      if (payload !== null) return payload;
      await this._wait(deadline);
      this._frames.feed(this._take());
      if (this._frames.frames.length > 64) this._frames.frames.splice(0, this._frames.frames.length - 64);
    }
  }
  _clear() { this._chunks = []; this._offset = 0; this._frames = new FrameReader(); }

  async _send(type, payload) {
    if (!this.connected || this._closing) throw this._failure || new Error('Connect first');
    await this._transmit(encodePacket(type, payload));
  }

  // Bytes for the ESC UART. Changing baud drops replies from the previous one.
  async _uart(data, baud = BAUD.APP) {
    if (baud !== this._baud) { this._baud = baud; this._clear(); }
    await this._send(PACKET.UART, [baud, ...data]);
  }

  async _command(type, payload = [], timeout = 1000) {
    const reply = new Promise((resolve, reject) => { this._reply = { resolve, reject }; });
    const timer = setTimeout(() => this._reply?.reject(new TimeoutError('Bridge did not reply')), timeout);
    try {
      await this._send(type, payload);
      const state = parseState(await reply);
      if (state.status !== 0) throw new Error(BRIDGE_ERRORS[state.status] || `Bridge error ${state.status}`);
      this.state = state;
      return state;
    } finally { clearTimeout(timer); this._reply = null; }
  }

  // Opening native USB can reset the bridge, so keep asking for a few seconds.
  async _hello() {
    const deadline = performance.now() + 3000;
    let state;
    while (!state) {
      try { state = await this._command(PACKET.HELLO, [], 400); }
      catch (error) {
        if (!(error instanceof TimeoutError)) throw error;
        if (performance.now() > deadline) {
          throw new Error('Bridge did not answer. Flash the bridge firmware from the Firmware tab, then connect again.');
        }
      }
    }
    if (state.version !== BRIDGE_VERSION) {
      throw new Error(`Bridge firmware version ${state.version} is not supported. Flash the bridge from the Firmware tab.`);
    }
    return state;
  }

  transaction(action) {
    const run = this._tail.then(() => {
      if (!this.connected || this._closing) throw this._failure || new Error('Connect first');
      return action({
        write: (data, baud) => this._uart(data, baud),
        read: (count, timeout) => this._readBytes(count, timeout),
        clear: () => this._clear(),
        readFrame: (address, command, timeout = 500) => this._readFrame(address, command, timeout),
        // Stale replies from an earlier timeout are dropped before each request.
        exchange: async (address, command, payload = new Uint8Array(), timeout = 500) => {
          this._clear();
          await this._uart(encodeFrame(address, command, payload));
          return this._readFrame(address, command, timeout);
        },
        command: (type, payload, timeout) => this._command(type, payload, timeout),
      });
    });
    this._tail = run.catch(() => {});
    return run;
  }

  exchange(address, command, payload = new Uint8Array(), timeout = 500) {
    return this.transaction(io => io.exchange(address, command, payload, timeout));
  }
  send(address, command, payload = new Uint8Array()) {
    return this.transaction(io => io.write(encodeFrame(address, command, payload)));
  }

  hello() { return this.transaction(io => io.command(PACKET.HELLO)); }
  setPins(rx, tx, gnd) { return this.transaction(io => io.command(PACKET.PINS, [rx & 255, tx & 255, gnd & 255])); }
  setAux(pin, pulse) { return this.transaction(io => io.command(PACKET.AUX, [pin & 255, pulse & 255, pulse >> 8])); }
  auxOff() { return this.setAux(this.state?.aux.pin ?? 0, 0); }

  // Telemetry: the bridge polls the ESC itself and sends stamped status frames.
  _telemetry(data) {
    if (!this.streaming) return;
    try {
      for (let offset = 0; offset < data.length;) {
        if (data.length - offset < 8) throw new Error('Incomplete telemetry');
        const stamp = new DataView(data.buffer, data.byteOffset + offset, 4).getUint32(0, true);
        const size = data[offset + 6] + 4;
        if (offset + 4 + size > data.length) throw new Error('Incomplete telemetry frame');
        const frame = decodeFrame(data.subarray(offset + 4, offset + 4 + size));
        if (frame.command !== 1 || frame.address !== this.streamAddress) throw new Error('Unexpected telemetry frame');
        this.sampleTime = this.lastStamp === undefined ? performance.now() : this.sampleTime + ((stamp - this.lastStamp) >>> 0) / 1000;
        this.lastStamp = stamp;
        this.samples.push({ status: unpackStatus(frame.payload), now: this.sampleTime });
        if (this.samples.length > 512) this.samples.shift();
        offset += 4 + size;
      }
    } catch { /* A damaged batch only costs its own samples. */ }
    this.sampleWake?.();
  }

  async startTelemetry(address, rate) {
    this.samples = [];
    this.lastStamp = undefined;
    this.streamAddress = address;
    this.streaming = true;
    try { await this.transaction(io => io.command(PACKET.STREAM, [address, Math.min(200, Math.round(rate))])); }
    catch (error) { this.streaming = false; throw error; }
  }

  async stopTelemetry() {
    this.streaming = false;
    this.sampleWake?.();
    this.samples = [];
    if (this.connected) await this.transaction(io => io.command(PACKET.STREAM, [0, 0]));
  }

  // The next sample, or null after the timeout or once telemetry stops.
  async nextTelemetry(timeout = 250) {
    if (!this.samples.length && this.streaming && !this._failure) {
      await new Promise(resolve => {
        const timer = setTimeout(() => { this.sampleWake = null; resolve(); }, timeout);
        this.sampleWake = () => { clearTimeout(timer); this.sampleWake = null; resolve(); };
      });
    }
    if (this._failure) throw this._failure;
    return this.samples.shift() ?? null;
  }
}
