import { encodeFrame, FrameReader } from './protocol.js';

export class TimeoutError extends Error {
  constructor(message = 'Serial response timed out') { super(message); this.name = 'TimeoutError'; }
}
const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
const encoder = new TextEncoder();
const decoder = new TextDecoder();

export class SerialConnection extends EventTarget {
  static owner = null;

  constructor() {
    super();
    this.port = null;
    this.connected = false;
    this._tail = Promise.resolve();
    this._chunks = [];
    this._offset = 0;
    this._waiter = null;
    this._frames = new FrameReader();
    this._prefix = null;
    this._lastWrite = 0;
  }

  async connect(port) {
    if (this.connected) throw new Error('Serial port is already connected');
    if (SerialConnection.owner) throw new Error('Another serial operation owns the port');
    SerialConnection.owner = this;
    try {
      this.port = port || await navigator.serial.requestPort();
      await this.port.open({ baudRate: 115200, bufferSize: 65536 });
      await this.port.setSignals({ dataTerminalReady: false, requestToSend: false });
      this._reader = this.port.readable.getReader();
      this._writer = this.port.writable.getWriter();
      this._failure = null;
      this._closing = false;
      this.connected = true;
      this._prefix = null;
      this._pump = this._read();
      return this;
    } catch (error) {
      if (this.port?.readable && !this._reader) await this.port.close();
      SerialConnection.owner = null;
      throw error;
    }
  }

  async _read() {
    try {
      while (this.connected) {
        const { value, done } = await this._reader.read();
        if (done) break;
        if (value?.length) { this._chunks.push(value); this._wake(); }
      }
      if (!this._closing) throw new Error('Serial port disconnected');
    } catch (error) {
      this._failure = error;
      this._wake();
      if (!this._closing) {
        this.connected = false;
        const event = new Event('disconnect'); event.error = error; this.dispatchEvent(event);
        // Close outside the read pump so closing can await this pump safely.
        queueMicrotask(() => { void this.disconnect().catch(closeError => { this._failure = closeError; }); });
      }
    }
  }

  disconnect() {
    if (this._closeTask) return this._closeTask;
    if (!this._reader) return Promise.resolve();
    this._closeTask = this._close();
    return this._closeTask.finally(() => { this._closeTask = null; });
  }
  async _close() {
    this._closing = true;
    this.connected = false;
    this._failure ||= new Error('Serial port closed');
    this._wake();
    try {
      await this._reader.cancel();
      await this._pump;
      await this._tail.catch(() => {});
    } finally {
      this._reader.releaseLock();
      this._writer.releaseLock();
      try { await this.port.close(); }
      finally {
        this._reader = null; this._writer = null;
        this._chunks = []; this._offset = 0; this._frames = new FrameReader();
        if (SerialConnection.owner === this) SerialConnection.owner = null;
      }
    }
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
  async _readLine(timeout = 1000) {
    const deadline = performance.now() + timeout;
    const bytes = [];
    while (true) {
      await this._wait(deadline);
      const chunk = this._chunks[0];
      const newline = chunk.indexOf(10, this._offset);
      const part = this._take(newline < 0 ? Infinity : newline - this._offset + 1);
      bytes.push(...part);
      if (newline >= 0) return decoder.decode(new Uint8Array(bytes)).trim();
      if (bytes.length > 8192) throw new Error('Bridge response exceeds 8192 bytes');
    }
  }
  async _write(data) {
    if (!this.connected || this._closing) throw this._failure || new Error('Connect a serial port');
    await this._writer.write(data);
    this._lastWrite = performance.now();
  }
  _clear() { this._chunks = []; this._offset = 0; this._frames = new FrameReader(); }
  async _readFrame(address, command, timeout) {
    const deadline = performance.now() + timeout;
    while (true) {
      const payload = this._frames.take(address, command);
      if (payload !== null) return payload;
      await this._wait(deadline);
      this._frames.feed(this._take());
      // Responses for other devices are bounded; a stale broadcast cannot grow memory indefinitely.
      if (this._frames.frames.length > 64) this._frames.frames.splice(0, this._frames.frames.length - 64);
    }
  }

  transaction(action) {
    const run = this._tail.then(() => {
      if (!this.connected || this._closing) throw this._failure || new Error('Connect a serial port');
      return action({
        write: data => this._write(data), read: (count, timeout) => this._readBytes(count, timeout),
        readLine: timeout => this._readLine(timeout), clear: () => this._clear(),
        readFrame: (address, command, timeout = 500) => this._readFrame(address, command, timeout),
        shell: (command, options) => this._shell(command, options),
        mode: mode => this._mode(mode),
        exchange: async (address, command, payload = new Uint8Array(), timeout = 500) => {
          await this._write(encodeFrame(address, command, payload)); return this._readFrame(address, command, timeout);
        },
      });
    });
    this._tail = run.catch(() => {});
    return run;
  }

  exchange(address, command, payload = new Uint8Array(), timeout = 500) {
    return this.transaction(io => io.exchange(address, command, payload, timeout));
  }
  send(address, command, payload = new Uint8Array()) { return this.transaction(io => io.write(encodeFrame(address, command, payload))); }

  async _sync() {
    for (const prefix of ['!#', '']) {
      await pause(Math.max(0, 26 - (performance.now() - this._lastWrite)));
      this._clear();
      await this._write(encoder.encode(`${prefix}ping\n`));
      const deadline = performance.now() + 350;
      try {
        while (performance.now() < deadline) {
          if (await this._readLine(deadline - performance.now()) === 'pong') { this._prefix = prefix; return; }
        }
      } catch (error) { if (!(error instanceof TimeoutError)) throw error; }
    }
    throw new Error('Bridge shell did not respond');
  }
  async _shell(command, { expected, timeout = 2000 } = {}) {
    if (/\r|\n/.test(command)) throw new Error('Enter one command at a time');
    if (this._prefix === null) await this._sync();
    await pause(Math.max(0, 26 - (performance.now() - this._lastWrite)));
    this._clear();
    await this._write(encoder.encode(`${this._prefix}${command}\n`));
    const lines = [];
    const deadline = performance.now() + timeout;
    while (performance.now() < deadline) {
      const line = await this._readLine(deadline - performance.now());
      if (!line) continue;
      lines.push(line);
      if (line === 'err' || line.startsWith('# error') || line.startsWith('err ') || line.startsWith('# pins error=')) throw new Error(line);
      if (expected) {
        if (typeof expected === 'function' ? expected(line) : line === expected) return lines.join('\n');
      } else {
        while (true) {
          try { const next = await this._readLine(15); if (next) lines.push(next); }
          catch (error) { if (error instanceof TimeoutError) return lines.join('\n'); throw error; }
        }
      }
    }
    throw new TimeoutError(`Bridge did not acknowledge ${command}`);
  }
  shell(command, options) { return this.transaction(() => this._shell(command, options)); }
  async _mode(mode) {
    if (!['app', 'rom', 'off', 'handoff', 'upload'].includes(mode)) throw new Error('Invalid bridge mode');
    await this._shell(`bridge ${mode}`, { expected: `# bridge=${mode}` });
    this._prefix = mode === 'off' ? null : '!#';
    this._clear();
  }
  mode(mode) { return this.transaction(() => this._mode(mode)); }
}
