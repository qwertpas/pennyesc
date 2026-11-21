import {FLASH_BASE, FLASH_PAGE_BYTES, loadEscImage} from './flash.js';

const ROM = 0x08000000;
const CALIBRATION = 0x08003d80;
const PECR = 0x40022004, PEKEYR = 0x4002200c, PRGKEYR = 0x40022010, SR = 0x40022018;
const FLASH_ERRORS = 0x32f00;
const LOCK = 7, OPTION_LOCK = 4, PROGRAM = 8, ERASE = 1 << 9, HALF_PAGE = 1 << 10;
const DHCSR = 0xe000edf0, DEMCR = 0xe000edfc, AIRCR = 0xe000ed0c, DEBUG_KEY = 0xa05f0000;
const V2 = [0x3748, 0x374b, 0x3752], V3 = [0x374e, 0x374f, 0x3753, 0x3754, 0x3757];
const word = value => Uint8Array.of(value, value >>> 8, value >>> 16, value >>> 24);

export function requestStLink() {
  return navigator.usb.requestDevice({
    filters: [...V2, ...V3].map(productId => ({vendorId: 0x0483, productId})),
  });
}

export async function flashStLink(device, boot, app, address, progress) {
  const bootBytes = FLASH_BASE - ROM;
  if (boot.length < 8 || boot.length > bootBytes) throw new Error('Invalid PennyESC bootloader size');
  const entry = new DataView(boot.buffer, boot.byteOffset, boot.length).getUint32(4, true);
  if (!(entry & 1) || entry < ROM || entry >= ROM + boot.length) throw new Error('Invalid PennyESC bootloader');
  const firmware = loadEscImage(app, address);
  if (FLASH_BASE + firmware.length > CALIBRATION) throw new Error('Firmware overlaps PennyESC calibration');
  const image = new Uint8Array(bootBytes + firmware.length);
  image.fill(255, boot.length, bootBytes);
  image.set(boot);
  image.set(firmware.subarray(0, app.length), bootBytes);

  const outEndpoint = device.productId === 0x3748 ? 2 : 1;
  let inDebug = false, needsLock = false, usbOk = true;

  async function exchange(bytes, length = 0, data) {
    const packet = new Uint8Array(16);
    packet.set(bytes);
    try {
      for (const payload of data ? [packet, data] : [packet]) {
        const result = await transfer(device.transferOut(outEndpoint, payload));
        if (result.status !== 'ok' || result.bytesWritten !== payload.length) throw new Error('ST-Link USB write failed');
      }
      if (length) {
        const result = await transfer(device.transferIn(1, length));
        if (result.status !== 'ok' || result.data?.byteLength !== length) throw new Error('ST-Link USB read failed');
        return result.data;
      }
    } catch (error) {
      usbOk = false;
      throw error;
    }
  }

  async function checked(bytes, length = 2) {
    const reply = await exchange(bytes, length);
    const status = reply.getUint8(0);
    if (status !== 0x80) throw new Error(`ST-Link command 0x${bytes[1].toString(16)} failed (0x${status.toString(16)})`);
    return reply;
  }

  const readDebug = async addr => (await checked([0xf2, 0x36, ...word(addr)], 8)).getUint32(4, true);
  const writeDebug = (addr, value) => checked([0xf2, 0x35, ...word(addr), ...word(value)]);
  const memory = (command, addr, size) => [0xf2, command, ...word(addr), size, 0];
  async function readMemory(addr, size) {
    const data = await exchange(memory(0x07, addr, size), size);
    await checked([0xf2, 0x3e], 12);
    return data;
  }
  async function writeMemory(addr, data) {
    await exchange(memory(0x08, addr, data.length), 0, data);
    await checked([0xf2, 0x3e], 12);
  }
  const readWord = async addr => (await readMemory(addr, 4)).getUint32(0, true);
  const writeWord = (addr, value) => writeMemory(addr, word(value));

  async function writeFlash(addr, data) {
    await writeMemory(addr, data);
    const deadline = performance.now() + 500;
    while (performance.now() < deadline) {
      const status = await readWord(SR);
      if (status & 1) continue;
      if (status & FLASH_ERRORS) throw new Error(`PennyESC flash error 0x${(status & FLASH_ERRORS).toString(16)} at 0x${addr.toString(16)}`);
      return;
    }
    throw new Error('PennyESC flash controller timed out');
  }

  try {
    await device.open();
    if (!device.configuration) await device.selectConfiguration(1);
    await device.claimInterface(0);
    const mode = (await exchange([0xf5], 2)).getUint8(0);
    if (mode === 0) await exchange([0xf3, 0x07]); // DFU exit
    if (mode === 2) await exchange([0xf2, 0x21]); // Debug exit
    if (mode === 3) await exchange([0xf4, 0x01]); // SWIM exit

    // Keep SWD slow enough for the STM32L0's reset clock.
    if (V3.includes(device.productId)) {
      const list = await checked([0xf2, 0x62], 52);
      const speeds = Array.from({length: Math.min(list.getUint8(8), 10)}, (_, i) => list.getUint32(12 + 4 * i, true));
      if (!speeds.length) throw new Error('ST-Link reported no SWD speeds');
      await checked([0xf2, 0x61, 0, 0, ...word(Math.min(...speeds))], 8);
    } else {
      await checked([0xf2, 0x43, 31]); // 125 kHz on V2/V2.1
    }
    await checked([0xf2, 0x30, 0xa3]); // Enter SWD without NRST
    inDebug = true;
    if (((await readWord(0x40015800)) & 0xfff) !== 0x457 ||
        ((await readWord(0x1ff8007c)) & 0xffff) !== 16) throw new Error('Expected a 16 KB STM32L01x PennyESC');
    await writeDebug(DHCSR, DEBUG_KEY | 3);
    if (!((await readDebug(DHCSR)) & (1 << 17))) throw new Error('Could not halt PennyESC');

    needsLock = true;
    await writeWord(SR, FLASH_ERRORS | 2);
    const locks = await readWord(PECR);
    if (locks & 1) {
      await writeWord(PEKEYR, 0x89abcdef);
      await writeWord(PEKEYR, 0x02030405);
    }
    if (locks & 2) {
      await writeWord(PRGKEYR, 0x8c9daebf);
      await writeWord(PRGKEYR, 0x13141516);
    }
    if ((await readWord(PECR)) & 3) throw new Error('PennyESC flash is locked');

    await writeWord(PECR, OPTION_LOCK | PROGRAM | ERASE);
    for (let offset = 0; offset < image.length; offset += FLASH_PAGE_BYTES) {
      progress('Erasing', 100 * offset / image.length);
      await writeFlash(ROM + offset, word(0));
    }
    // Each aligned SWD write supplies all 16 words of a half-page while the core is halted.
    await writeWord(PECR, OPTION_LOCK | PROGRAM | HALF_PAGE);
    for (let offset = 0; offset < image.length; offset += FLASH_PAGE_BYTES / 2) {
      progress('Writing', 100 * offset / image.length);
      await writeFlash(ROM + offset, image.subarray(offset, offset + FLASH_PAGE_BYTES / 2));
    }
    await writeWord(PECR, LOCK);
    needsLock = false;

    for (let offset = 0; offset < image.length; offset += FLASH_PAGE_BYTES) {
      progress('Verifying', 100 * offset / image.length);
      const page = await readMemory(ROM + offset, FLASH_PAGE_BYTES);
      for (let i = 0; i < page.byteLength; i++) {
        if (page.getUint8(i) !== image[offset + i]) throw new Error(`ST-Link verification failed at 0x${(ROM + offset + i).toString(16)}`);
      }
    }
    await writeDebug(DEMCR, 0); // Clear reset vector catch.
    await writeDebug(AIRCR, 0x05fa0004); // Reset via SYSRESETREQ.
    await writeDebug(DHCSR, DEBUG_KEY | 1);
    progress('Verified', 100);
  } finally {
    if (needsLock && usbOk) await writeWord(PECR, LOCK).catch(() => {});
    if (inDebug && usbOk) await exchange([0xf2, 0x21]).catch(() => {});
    if (device.opened) await device.close().catch(() => {});
  }
}

async function transfer(pending) {
  let timer;
  const timeout = new Promise((_, reject) => {
    timer = setTimeout(() => reject(new Error('ST-Link USB transfer timed out')), 3000);
  });
  try { return await Promise.race([pending, timeout]); }
  finally { clearTimeout(timer); }
}
