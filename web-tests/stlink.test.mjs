import test from 'node:test';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {readFile} from 'node:fs/promises';
import {flashStLink} from '../docs/stlink.js';

const firmware = new URL('../docs/firmware/', import.meta.url);
const {pennyesc} = JSON.parse(await readFile(new URL('manifest.json', firmware)));
const [boot, app] = await Promise.all([pennyesc.bootloader, pennyesc].map(async asset => {
  const data = await readFile(new URL(asset.path, firmware));
  assert.equal(data.length, asset.bytes);
  assert.equal(createHash('sha256').update(data).digest('hex'), asset.sha256);
  return data;
}));

function probe(productId, {chip = 0x457, corrupt = false} = {}) {
  const flash = Buffer.alloc(16 * 1024, 0xa5);
  const registers = new Map([[0x40015800, chip], [0x1ff8007c, 16], [0x40022004, 7], [0x40022018, 0]]);
  let reply, write, halted = false, debug = false;
  return {
    productId, flash, resets: 0, erased: [],
    async open() { this.opened = true; },
    async selectConfiguration(number) { assert.equal(number, 1); this.configuration = {}; },
    async claimInterface(number) { assert.equal(number, 0); },
    async close() { this.opened = false; },
    async transferOut(endpoint, bytes) {
      assert.equal(endpoint, productId === 0x3748 ? 2 : 1);
      assert.ok(this.opened);
      const data = Buffer.from(bytes);
      if (write) {
        const [addr, size] = write;
        assert.equal(data.length, size);
        if (addr < 0x08000000 || addr >= 0x08004000) {
          assert.equal(size, 4);
          assert.ok(halted);
          const value = data.readUInt32LE();
          if (addr === 0x4002200c && value === 0x02030405) registers.set(0x40022004, 6);
          if (addr === 0x40022010 && value === 0x13141516) registers.set(0x40022004, 4);
          if (addr === 0x40022004) registers.set(addr, value);
        } else {
          const offset = addr - 0x08000000;
          const mode = registers.get(0x40022004);
          assert.ok(halted && offset + size <= 0x3d80);
          if (mode === 0x20c) {
            assert.equal(offset % 128, 0);
            assert.deepEqual(data, Buffer.alloc(4));
            flash.fill(0, offset, offset + 128);
            this.erased.push(offset);
          } else {
            assert.equal(mode, 0x40c);
            assert.equal(size, 64);
            assert.equal(offset % 64, 0);
            assert.ok(flash.subarray(offset, offset + size).every(byte => byte === 0));
            data.copy(flash, offset);
          }
        }
        write = undefined;
      } else {
        assert.equal(data.length, 16);
        const [group, command] = data;
        const addr = data.readUInt32LE(2), size = data.readUInt16LE(6);
        if (group === 0xf5) reply = Buffer.of(2, 0);
        else {
          assert.equal(group, 0xf2);
          reply = Buffer.of(0x80, 0);
          switch (command) {
            case 0x21: debug = false; reply = undefined; break;
            case 0x30: assert.equal(data[2], 0xa3); debug = true; break;
            case 0x43: assert.equal(addr, 31); break;
            case 0x61: assert.equal(data.readUInt32LE(4), 200); reply = Buffer.alloc(8); reply[0] = 0x80; break;
            case 0x62:
              reply = Buffer.alloc(52);
              reply[0] = 0x80; reply[8] = 2; reply.writeUInt32LE(1000, 12); reply.writeUInt32LE(200, 16);
              break;
            case 0x07:
              assert.ok(debug);
              if (addr >= 0x08000000 && addr < 0x08004000) {
                assert.equal(registers.get(0x40022004), 7);
                reply = Buffer.from(flash.subarray(addr - 0x08000000, addr - 0x08000000 + size));
                if (corrupt) reply[0] ^= 1;
              } else {
                reply = Buffer.alloc(4); reply.writeUInt32LE(registers.get(addr));
              }
              break;
            case 0x08: assert.ok(debug); write = [addr, size]; break;
            case 0x35:
              if (addr === 0xe000edf0) halted = Boolean(data.readUInt32LE(6) & 2);
              if (addr === 0xe000ed0c) { assert.equal(data.readUInt32LE(6), 0x05fa0004); this.resets++; }
              break;
            case 0x36:
              assert.equal(addr, 0xe000edf0);
              reply = Buffer.alloc(8); reply[0] = 0x80; reply.writeUInt32LE(halted ? 1 << 17 : 0, 4);
              break;
            case 0x3e: reply = Buffer.alloc(12); reply[0] = 0x80; break;
            default: assert.fail(`Unexpected command ${command.toString(16)}`);
          }
        }
      }
      return {status: 'ok', bytesWritten: bytes.length};
    },
    async transferIn(endpoint, length) {
      assert.equal(endpoint, 1);
      assert.equal(reply.length, length);
      return {status: 'ok', data: new DataView(reply.buffer, reply.byteOffset, length)};
    },
  };
}

test('ST-Link seeds and verifies the bundled firmware at the selected address, preserving calibration', async () => {
  for (const productId of [0x3748, 0x374f]) {
    const device = probe(productId), stages = new Map();
    await flashStLink(device, boot, app, 9, (stage, percent) => stages.set(stage, percent));
    const expected = Buffer.concat([boot, Buffer.alloc(0x580 - boot.length, 255), app]);
    expected[expected.indexOf('PNYADDR') + 7] = 9;
    const pages = Math.ceil(expected.length / 128);
    assert.deepEqual(device.erased, Array.from({length: pages}, (_, page) => page * 128));
    assert.deepEqual(device.flash.subarray(0, expected.length), expected);
    assert.ok(device.flash.subarray(0x3d80).every(byte => byte === 0xa5));
    assert.deepEqual([...stages.keys()], ['Erasing', 'Writing', 'Verifying', 'Verified']);
    assert.equal(stages.get('Verified'), 100);
    assert.equal(device.resets, 1);
    assert.equal(device.opened, false);
  }
});

test('ST-Link rejects the wrong chip or corrupted read-back and releases the device without a reset', async () => {
  for (const [options, error] of [[{chip: 0x425}, /Expected a 16 KB/], [{corrupt: true}, /verification failed/]]) {
    const device = probe(0x3748, options);
    await assert.rejects(flashStLink(device, boot, app, 1, () => {}), error);
    if (!options.corrupt) assert.equal(device.erased.length, 0);
    assert.equal(device.resets, 0);
    assert.equal(device.opened, false);
  }
});
