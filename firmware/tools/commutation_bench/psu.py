"""Minimal SKY TOPPOWER STP2S Modbus RTU client (read + write single register)."""
import os, struct, sys, time
import serial

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack('<H', crc)

class PSU:
    def __init__(self, port=os.environ.get('PSU_PORT', '/dev/cu.usbserial-110')):
        self.s = serial.Serial(port, 9600, timeout=0.3)
    def _x(self, pdu, n):
        frame = bytes([1]) + pdu
        self.s.reset_input_buffer(); self.s.write(frame + crc16(frame))
        r = self.s.read(n)
        if len(r) != n or crc16(r[:-2]) != r[-2:]:
            raise IOError(f'bad reply {r.hex()}')
        return r
    def read(self, reg, count=1):
        r = self._x(struct.pack('>BHH', 3, reg, count), 5 + 2 * count)
        return list(struct.unpack('>' + 'H' * count, r[3:3 + 2 * count]))
    def write(self, reg, value):
        self._x(struct.pack('>BHH', 6, reg, value), 8)
    def snapshot(self):
        dp = self.read(0x0005)[0]
        vd, ad = (dp >> 8) & 0xF, (dp >> 4) & 0xF
        v, a = self.read(0x0010, 2)
        sv, sa = self.read(0x0030, 2)
        out, = self.read(0x0001)
        return dict(on=out, v=v / 10**vd, a=a / 10**ad, set_v=sv / 10**vd, set_a=sa / 10**ad, dp=hex(dp))

if __name__ == '__main__':
    p = PSU()
    print(p.snapshot())
