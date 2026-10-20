import { BOOT_MAGIC, CMD, checkResult, encodeFrame, pack, unpackStatus } from './protocol.js';
import { SerialConnection, TimeoutError } from './serial.js';

export const FLASH_BASE = 0x08000580;
export const FLASH_APP_BYTES = 14336;
export const FLASH_PAGE_BYTES = 128;
const ACK = 0x79;
const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
const xor = bytes => bytes.reduce((value, byte) => value ^ byte, 0);
const same = (left, right) => left.length === right.length && left.every((value, i) => value === right[i]);

export function loadEscImage(image) {
  if (!(image instanceof Uint8Array) || image.length < 8) throw new Error('Select a PennyESC application binary');
  const length = Math.ceil(image.length / FLASH_PAGE_BYTES) * FLASH_PAGE_BYTES;
  if (length > FLASH_APP_BYTES) throw new Error(`Application exceeds ${FLASH_APP_BYTES} bytes`);
  const view = new DataView(image.buffer, image.byteOffset, image.byteLength);
  const stack = view.getUint32(0, true), entry = view.getUint32(4, true);
  if ((stack & 0x2ff80000) !== 0x20000000 || !(entry & 1) || entry < FLASH_BASE || entry >= FLASH_BASE + FLASH_APP_BYTES) throw new Error('Binary is not a PennyESC application at 0x08000580');
  const padded = new Uint8Array(length).fill(255); padded.set(image);
  return padded;
}

export class Stm32Bootloader {
  constructor(io) { this.io = io; }
  async ack(timeout = 1000) {
    const value = (await this.io.read(1, timeout))[0];
    if (value !== ACK) throw new Error(value === 0x1f ? 'Bootloader returned NACK' : `Unexpected bootloader byte 0x${value.toString(16)}`);
  }
  async command(command) { await this.io.write(new Uint8Array([command, command ^ 255])); await this.ack(); }
  async address(address) {
    const data = new Uint8Array(5);
    new DataView(data.buffer).setUint32(0, address, false); data[4] = xor(data.subarray(0, 4));
    await this.io.write(data); await this.ack();
  }
  async sync(timeout = 1500) {
    this.io.clear();
    const deadline = performance.now() + timeout;
    while (performance.now() < deadline) {
      await this.io.write(new Uint8Array([0x7f]));
      try { await this.ack(Math.min(180, deadline - performance.now())); return; }
      catch (error) {
        if (!(error instanceof TimeoutError) && error.message !== 'Bootloader returned NACK') throw error;
      }
    }
    throw new Error('Bootloader did not respond; reset the target and use Recover');
  }
  async erase(pages) {
    if (!pages.length || pages.some(page => !Number.isInteger(page) || page < 0 || page >= FLASH_APP_BYTES / FLASH_PAGE_BYTES)) throw new Error('Erase exceeds application region');
    await this.command(0x44);
    const data = new Uint8Array(3 + pages.length * 2);
    const view = new DataView(data.buffer);
    view.setUint16(0, pages.length - 1, false);
    pages.forEach((page, i) => view.setUint16(2 + i * 2, page, false));
    data[data.length - 1] = xor(data.subarray(0, -1));
    await this.io.write(data); await this.ack(20000);
  }
  async write(address, bytes) {
    if (!bytes.length || bytes.length > 256 || bytes.length % 64 || address % 64 || address < FLASH_BASE || address + bytes.length > FLASH_BASE + FLASH_APP_BYTES) throw new Error('Write exceeds application region or alignment');
    await this.command(0x31); await this.address(address);
    const data = new Uint8Array(bytes.length + 2);
    data[0] = bytes.length - 1; data.set(bytes, 1); data[data.length - 1] = xor(data.subarray(0, -1));
    await this.io.write(data); await this.ack();
  }
  async read(address, length) {
    if (length < 1 || length > 256 || address < FLASH_BASE || address + length > FLASH_BASE + FLASH_APP_BYTES) throw new Error('Read exceeds application region');
    await this.command(0x11); await this.address(address);
    await this.io.write(new Uint8Array([length - 1, (length - 1) ^ 255])); await this.ack();
    return this.io.read(length, 1500);
  }
  async go() { await this.command(0x21); await this.address(FLASH_BASE); }
}

export async function programEsc(boot, padded, progress = () => {}) {
  if (!padded.length || padded.length > FLASH_APP_BYTES || padded.length % FLASH_PAGE_BYTES) throw new Error('Invalid application size');
  const pages = Array.from({ length: padded.length / FLASH_PAGE_BYTES }, (_, i) => i);
  progress('erase', 0);
  for (let start = 0; start < pages.length; start += 8) {
    await boot.erase(pages.slice(start, start + 8)); progress('erase', Math.min(100, (start + 8) / pages.length * 100));
  }
  for (let offset = 0; offset < padded.length; offset += FLASH_PAGE_BYTES) {
    await boot.write(FLASH_BASE + offset, padded.subarray(offset, offset + FLASH_PAGE_BYTES));
    progress('write', (offset + FLASH_PAGE_BYTES) / padded.length * 100);
  }
  for (let offset = 0; offset < padded.length; offset += FLASH_PAGE_BYTES) {
    const expected = padded.subarray(offset, offset + FLASH_PAGE_BYTES);
    if (!same(await boot.read(FLASH_BASE + offset, expected.length), expected)) throw new Error(`Readback verification failed at 0x${(FLASH_BASE + offset).toString(16)}`);
    progress('verify', (offset + FLASH_PAGE_BYTES) / padded.length * 100);
  }
}

export async function flashEsc(client, image, { recover = false, verifyAddress = client.address, progress = () => {}, onLog = () => {}, onResetNeeded = () => {} } = {}) {
  const padded = loadEscImage(image);
  return client.connection.transaction(async io => {
    progress('connect', 0);
    await io.mode('handoff');
    const found = [];
    for (let address = 0; address < 16; address++) {
      io.clear();
      try { await io.exchange(address, CMD.GET_STATUS, new Uint8Array(), 35); found.push(address); }
      catch (error) { if (!(error instanceof TimeoutError)) throw error; }
    }
    const quiet = found.filter(address => recover || address !== client.address);
    for (const address of quiet) {
      const result = await io.exchange(address, CMD.SET_QUIET, pack('H', 60000));
      if (result.length !== 1) throw new Error(`Invalid quiet response from ${address}`);
      checkResult(result[0]);
    }
    if (quiet.length) onLog(`Other addresses quiet for 60 s: ${quiet.join(', ')}`);
    if (!recover) {
      if (!found.includes(client.address)) throw new Error(`Address ${client.address} did not respond; use Recover after resetting it`);
      const result = await io.exchange(client.address, CMD.ENTER_BOOT, pack('I', BOOT_MAGIC), 500);
      if (result.length !== 1) throw new Error('Invalid boot response');
      checkResult(result[0]);
    }
    await io.mode('upload');
    const boot = new Stm32Bootloader(io);
    if (recover) onResetNeeded();
    await boot.sync(recover ? 8000 : 1500);
    await programEsc(boot, padded, progress);
    await boot.go();
    await io.mode('handoff');
    const deadline = performance.now() + 8000;
    while (performance.now() < deadline) {
      try {
        const status = unpackStatus(await io.exchange(verifyAddress, CMD.GET_STATUS, new Uint8Array(), 250));
        checkResult(status.result);
        progress('done', 100); onLog(`Verified ${image.length} bytes`); return status;
      } catch (error) {
        if (!(error instanceof TimeoutError)) throw error;
        await pause(100);
      }
    }
    throw new Error(`Firmware verified; address ${verifyAddress} did not resume`);
  });
}

export async function fetchBinary(path, base = new URL('./firmware/', import.meta.url)) {
  const response = await fetch(new URL(path, base));
  if (!response.ok) throw new Error(`Firmware download failed (${response.status})`);
  return new Uint8Array(await response.arrayBuffer());
}

export async function flashBridge(connection, board, { progress = () => {}, onLog = () => {} } = {}) {
  if (!board?.flash?.length) throw new Error('Select a bridge board');
  const port = connection.connected ? connection.port : await navigator.serial.requestPort();
  progress('download', 0);
  const files = await Promise.all(board.flash.map(async file => ({ address: file.address, data: await fetchBinary(file.path) })));
  await connection.disconnect();
  if (SerialConnection.owner) throw new Error('Another serial operation owns the port');
  const owner = {};
  SerialConnection.owner = owner;
  let transport;
  try {
    const { ESPLoader, Transport } = await import('./vendor/esptool-js-0.7.0.js');
    transport = new Transport(port, false);
    const loader = new ESPLoader({ transport, baudrate: 460800, terminal: { clean() {}, write: onLog, writeLine: onLog }, debugLogging: false });
    progress('connect', 0);
    const detected = await loader.main();
    const normalize = name => name.toLowerCase().replace(/[^a-z0-9]/g, '');
    if (normalize(loader.chip.CHIP_NAME) !== normalize(board.chip)) throw new Error(`Selected ${board.name}; connected chip is ${detected}`);
    await loader.writeFlash({ fileArray: files, flashSize: 'keep', flashMode: 'keep', flashFreq: 'keep', eraseAll: false, compress: true,
      reportProgress: (index, written, total) => progress('write', (index + written / total) / files.length * 100),
    });
    for (let i = 0; i < files.length; i++) {
      const file = files[i];
      const actual = await loader.readFlash(file.address, file.data.length, (_, written, total) => progress('verify', (i + written / total) / files.length * 100));
      if (!same(actual, file.data)) throw new Error(`Bridge readback verification failed at 0x${file.address.toString(16)}`);
    }
    await loader.after('hard_reset');
    progress('done', 100); onLog(`Verified ${board.name}`);
    connection.port = port;
  } finally {
    try { if (transport && port.readable) await transport.disconnect(); }
    finally { if (SerialConnection.owner === owner) SerialConnection.owner = null; }
  }
}
