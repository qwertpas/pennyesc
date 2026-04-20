import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pennycal import (  # noqa: E402
    CAL_POINTS_PER_SWEEP,
    CAL_TOTAL_POINTS,
    CapturePoint,
    CalibrationError,
    LUT_SIZE,
    pseudo_index,
    solve_capture,
    solve_legacy_csv,
)


class SolverTests(unittest.TestCase):
    def test_solver_replays_filtered_csvs(self) -> None:
        data_dir = Path(__file__).resolve().parents[2] / "firmware" / "pennyesc_libopencm3" / "data"
        for name in (
            "MCUViewer_log_190627_20260305_filt.csv",
            "testrun59_filt.csv",
            "testrun59_6-5V_filt.csv",
        ):
            solved = solve_legacy_csv(data_dir / name)
            self.assertLessEqual(solved.fit_max_error_deg, 8.0, name)
            self.assertEqual(len(solved.angle_lut), LUT_SIZE)

    def test_capture_solver_rejects_large_sweep_delta(self) -> None:
        points = []
        radius = 20000.0
        for step in range(CAL_POINTS_PER_SWEEP):
            angle = 2.0 * math.pi * step / CAL_POINTS_PER_SWEEP
            x = int(round(radius * math.cos(angle)))
            y = int(round(radius * math.sin(angle)))
            points.append(CapturePoint(step, step, 0, x, y, 0, 0, 0))

        for step in range(CAL_POINTS_PER_SWEEP):
            angle = 2.0 * math.pi * step / CAL_POINTS_PER_SWEEP + math.radians(15.0)
            x = int(round(radius * math.cos(angle)))
            y = int(round(radius * math.sin(angle)))
            points.append(CapturePoint(CAL_POINTS_PER_SWEEP + step, step, 1, x, y, 0, 0, 0))

        with self.assertRaises(CalibrationError):
            solve_capture(points)

    def test_pseudo_index_range(self) -> None:
        values = {pseudo_index(1000, 0), pseudo_index(0, 1000), pseudo_index(-1000, 0), pseudo_index(0, -1000)}
        self.assertTrue(all(0 <= value < LUT_SIZE for value in values))
        self.assertEqual(len(values), 4)


if __name__ == "__main__":
    unittest.main()
