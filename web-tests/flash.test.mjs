import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { FLASH_BASE, FLASH_APP_BYTES, Stm32Bootloader, flashBridge, flashEsc, loadEscImage } from '../docs/flash.js';
import { SerialConnection } from '../docs/serial.js';
import { PennyClient, decodeFrame, encodeFrame } from '../docs/protocol.js';
import { FakePort, RomDevice, statusPayload } from './serial-device.mjs';
import { ESPLoader } from '../docs/vendor/esptool-js-0.7.0.js';

function image(length = 1100) {
  const bytes = Uint8Array.from({ length }, (_, i) => i & 255);
  const view = new DataView(bytes.buffer);
  view.setUint32(0, 0x20000800, true); view.setUint32(4, FLASH_BASE + 0x101, true);
  bytes.set(new TextEncoder().encode('PNYADDR\x01'), 100);
  return bytes;
}

function bridgePort(rom, { recover = false } = {}) {
  const encoder = new TextEncoder(), decoder = new TextDecoder();
  let mode = 'app';
  const quiet = [], modes = [];
  const port = new FakePort(data => {
    if (data[0] === 33 && data[1] === 35) {
      const text = decoder.decode(data).trim().slice(2);
      if (text === 'ping') return encoder.encode('pong\n');
      if (text.startsWith('bridge ')) {
        mode = text.slice(7); modes.push(mode);
        return encoder.encode(`# bridge=${mode}\n`);
      }
      throw new Error(`Unknown shell command ${text}`);
    }
    if (mode === 'upload') return rom.write(data);
    const frame = decodeFrame(data);
    const address = rom.running ? rom.memory[0x580 + 107] : 1;
    if (![0, address, 2].includes(frame.address) || (!rom.running && recover && frame.address === 1)) return null;
    if (frame.command === 1) return encodeFrame(frame.address, 1, statusPayload());
    if (frame.command === 13) { quiet.push(frame.address); return encodeFrame(frame.address, 13, new Uint8Array([0])); }
    if (frame.command === 11) return encodeFrame(frame.address, 11, new Uint8Array([0]));
    throw new Error('Unknown frame');
  }, { fragment: 7 });
  return { port, quiet, modes };
}

test('application validation rejects oversized, bootloader, and non-app binaries before writing', async () => {
  assert.equal(loadEscImage(image()).length, 1152);
  assert.throws(() => loadEscImage(image(FLASH_APP_BYTES + 1)), /exceeds/);
  assert.throws(() => loadEscImage(new Uint8Array(100)), /not a PennyESC/);
  const bytes = image(); new DataView(bytes.buffer).setUint32(4, 0x08000001, true);
  assert.throws(() => loadEscImage(bytes), /not a PennyESC/);
  const bundled = new Uint8Array(await readFile(new URL('../docs/firmware/pennyesc.bin', import.meta.url)));
  assert.ok(loadEscImage(bundled).length <= FLASH_APP_BYTES);
});

test('normal flashing erases only app pages, verifies every byte, preserves boot and calibration, and resumes app', async () => {
  const rom = new RomDevice(), setup = bridgePort(rom);
  const connection = new SerialConnection(); await connection.connect(setup.port);
  const data = image(), padded = loadEscImage(data), stages = [];
  try {
    const status = await flashEsc(new PennyClient(connection, 1), data, { progress: (stage, percent) => stages.push([stage, percent]) });
    assert.equal(status.x, 123);
    assert.deepEqual(rom.memory.slice(0x580, 0x580 + padded.length), padded);
    assert.ok(rom.memory.slice(0, 0x580).every(byte => byte === 0x5a));
    assert.ok(rom.memory.slice(0x3d80).every(byte => byte === 0x5a));
    assert.deepEqual(rom.erased, Array.from({ length: 9 }, (_, i) => i));
    assert.deepEqual(rom.written, rom.reads);
    assert.deepEqual(setup.quiet, [0, 2]);
    assert.deepEqual(setup.modes, ['handoff', 'upload', 'handoff', 'app']);
    assert.ok(rom.running);
    assert.deepEqual(stages.at(-1), ['done', 100]);
  } finally { await connection.disconnect(); }
});

test('the bundled image supports all addresses without changing any other firmware bytes', async () => {
  const original = new Uint8Array(await readFile(new URL('../docs/firmware/pennyesc.bin', import.meta.url)));
  const offset = Buffer.from(original).indexOf('PNYADDR') + 7;
  const padded = loadEscImage(original);
  for (let address = 0; address < 16; address++) {
    const result = loadEscImage(original, address);
    assert.equal(result[offset], address);
    result[offset] = padded[offset];
    assert.deepEqual(result, padded);
  }
  assert.equal(original[offset], 1);
  assert.throws(() => loadEscImage(original, 16), /Address must/);
  const missing = image(); missing[100] = 0;
  assert.throws(() => loadEscImage(missing, 4), /does not support address/);
  const duplicate = image(); duplicate.set(new TextEncoder().encode('PNYADDR\x01'), 200);
  assert.throws(() => loadEscImage(duplicate, 4), /does not support address/);
});

test('flashing changes address 1 to 4 and verifies the patched image and new address', async () => {
  const rom = new RomDevice(), setup = bridgePort(rom);
  const connection = new SerialConnection(); await connection.connect(setup.port);
  try {
    await flashEsc(new PennyClient(connection, 1), image(), {newAddress: 4});
    assert.equal(rom.memory[0x580 + 107], 4);
    assert.ok(rom.running);
    assert.equal((await new PennyClient(connection, 4).status()).calibrated, true);
    await assert.rejects(new PennyClient(connection, 1).status(5));
    assert.ok(rom.memory.slice(0x3d80).every(byte => byte === 0x5a));
    assert.equal(setup.modes.at(-1), 'app');
  } finally { await connection.disconnect(); }
});

test('an occupied new address prevents erase and programming', async () => {
  const rom = new RomDevice(), setup = bridgePort(rom);
  const connection = new SerialConnection(); await connection.connect(setup.port);
  try {
    await assert.rejects(flashEsc(new PennyClient(connection, 1), image(), {newAddress: 2}), /already in use/);
    assert.deepEqual(rom.erased, []);
    assert.deepEqual(rom.written, []);
  } finally { await connection.disconnect(); }
});

test('recovery accepts absent app and waits in upload mode for target reset', async () => {
  const rom = new RomDevice(), setup = bridgePort(rom, { recover: true });
  const connection = new SerialConnection(); await connection.connect(setup.port);
  let resetRequested = false;
  try {
    await flashEsc(new PennyClient(connection, 1), image(200), { recover: true, onResetNeeded: () => { resetRequested = true; } });
    assert.equal(resetRequested, true); assert.deepEqual(setup.quiet, [0, 2]); assert.equal(rom.running, true);
  } finally { await connection.disconnect(); }
});

test('verification failure never runs the unverified application', async () => {
  const rom = new RomDevice(); rom.corruptRead = true;
  const { port } = bridgePort(rom);
  const connection = new SerialConnection(); await connection.connect(port);
  try {
    await assert.rejects(() => flashEsc(new PennyClient(connection, 1), image(200)), /Readback verification failed/);
    assert.equal(rom.running, false);
  } finally { await connection.disconnect(); }
});

test('ROM primitives reject calibration writes and mass erases', async () => {
  const boot = new Stm32Bootloader({ write() { throw new Error('Must not write'); } });
  await assert.rejects(() => boot.erase([112]), /application region/);
  await assert.rejects(() => boot.write(FLASH_BASE + FLASH_APP_BYTES, new Uint8Array(128)), /application region/);
  await assert.rejects(() => boot.read(FLASH_BASE - 1, 128), /application region/);
});

test('ESP flasher releases live reader before esptool, verifies files and closes before reconnect', async t => {
  const port = new FakePort(() => null), connection = new SerialConnection();
  await connection.connect(port);
  const bytes = new Uint8Array([1, 2, 3, 4]);
  const saved = {};
  let reset = false;
  for (const method of ['main', 'writeFlash', 'readFlash', 'after']) saved[method] = ESPLoader.prototype[method];
  const originalFetch = globalThis.fetch;
  t.after(() => { for (const method of Object.keys(saved)) ESPLoader.prototype[method] = saved[method]; globalThis.fetch = originalFetch; });
  globalThis.fetch = async (url, options) => {
    assert.equal(options.cache, 'no-store');
    return new Response(bytes);
  };
  ESPLoader.prototype.main = async function () {
    assert.equal(this.serialOptions.bufferSize, 65536);
    assert.equal(port.closeCount, 1);
    assert.equal(connection.connected, false);
    assert.equal(port.readable, null);
    await port.open(); this.chip = { CHIP_NAME: 'ESP32-S3' }; return 'ESP32-S3';
  };
  ESPLoader.prototype.writeFlash = async function (options) {
    assert.equal(options.eraseAll, false); assert.equal(options.compress, true);
    assert.deepEqual(options.fileArray[0].data, bytes);
    assert.equal(options.fileArray[0].address, 65536);
  };
  ESPLoader.prototype.readFlash = async () => bytes;
  ESPLoader.prototype.after = async mode => { assert.equal(mode, 'hard_reset'); reset = true; };
  await flashBridge(connection, { name: 'XIAO', chip: 'ESP32-S3', flash: [{ path: 'firmware.bin', address: 65536 }] });
  assert.equal(reset, true); assert.equal(port.closeCount, 2); assert.equal(SerialConnection.owner, null);
  assert.equal(connection.connected, false);
  await connection.connect(port); await connection.disconnect();
});

test('disconnected ESP flashing selects a fresh port before starting network downloads', async t => {
  const connection = new SerialConnection(); connection.port = { stale: true };
  const descriptor = Object.getOwnPropertyDescriptor(globalThis, 'navigator');
  const originalFetch = globalThis.fetch;
  let selected = false;
  Object.defineProperty(globalThis, 'navigator', { configurable: true, value: { serial: { requestPort: async () => { selected = true; return {}; } } } });
  globalThis.fetch = async () => { assert.equal(selected, true); throw new Error('Download interrupted'); };
  t.after(() => {
    Object.defineProperty(globalThis, 'navigator', descriptor);
    globalThis.fetch = originalFetch;
  });
  await assert.rejects(() => flashBridge(connection, { flash: [{ path: 'firmware.bin', address: 0 }] }), /Download interrupted/);
  assert.equal(selected, true);
  assert.equal(SerialConnection.owner, null);
});
