import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pny_speed import check_speed


class SpeedTests(unittest.TestCase):
    def rows(self, speeds, duty=500):
        return [dict(velocity_turn32_per_s=rpm * 65536 / 60,
                     mode=1, duty=duty, faults=0) for rpm in speeds]

    def test_persistent_collapse_both_directions(self):
        for sign in (-1, 1):
            rows = self.rows([sign * v for v in [10000, 40000, 30000, 32000, 33000]], sign * 500)
            self.assertIsNone(check_speed(rows[:-1], sign * 500, 48000))
            self.assertEqual(check_speed(rows, sign * 500, 48000), "speed collapse")

    def test_speed_above_compact_capture_range(self):
        rows = self.rows([30000, 35000, 36000, 35000])
        self.assertIsNone(check_speed(rows, 500, 48000))
        self.assertEqual(check_speed(rows, 500, 34000), "speed limit")

    def test_output_stop_without_fault_flag(self):
        rows = self.rows([20000, 0])
        rows[-1].update(mode=0, duty=0)
        self.assertEqual(check_speed(rows, 500, 48000), "output stopped")
