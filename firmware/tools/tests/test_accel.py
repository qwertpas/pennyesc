import sys
from pathlib import Path
import struct
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pny_accel import acceleration, read_capture, speed_collapsed
from pnyproto import CMD_DEBUG, DEBUG_CAPTURE_READ


class AccelerationTests(unittest.TestCase):
    def test_constant_acceleration_both_directions(self):
        for direction in (-1, 1):
            samples = [(0, direction * i * 125) for i in range(200)]
            bands = acceleration(samples, direction * 400)
            self.assertEqual(len(bands), 5)
            for value in bands.values():
                self.assertAlmostEqual(value, 125000)

    def test_plateau_is_not_an_acceleration_measurement(self):
        samples = [(0, min(i * 200, 15000)) for i in range(200)]
        bands = acceleration(samples, 400)
        self.assertNotIn("14000-18000", bands)
        self.assertAlmostEqual(bands["6000-10000"], 200000)

    def test_single_spike_does_not_cross_band(self):
        samples = [(0, 1000) for _ in range(30)]
        samples[15] = (0, 32767)
        self.assertEqual(acceleration(samples, 400), {})

    def test_collapse_invalidates_both_directions(self):
        for sign in (-1, 1):
            samples = [(0, sign * v) for v in [0, 10000, 18000, 15000, 11000]]
            self.assertTrue(speed_collapsed(samples, sign * 400))
            self.assertFalse(speed_collapsed(samples[:3], sign * 400))

    def test_capture_block_offsets_and_signed_rpm(self):
        class Client:
            def exchange(self, command, payload):
                self.command = command
                subcmd, offset, count = struct.unpack("<BHB", payload)
                return (struct.pack("<BBHB", subcmd, 0, offset, count) +
                        b"".join(struct.pack("<Hh", i, -i) for i in range(offset, offset + count)))
        client = Client()
        self.assertEqual(read_capture(client, 31), [(i, -i) for i in range(31)])
        self.assertEqual(client.command, CMD_DEBUG)

    def test_capture_rejects_wrong_offset(self):
        class Client:
            def exchange(self, command, payload):
                return struct.pack("<BBHBHh", DEBUG_CAPTURE_READ, 0, 1, 1, 12, 345)
        with self.assertRaises(RuntimeError):
            read_capture(Client(), 1)
