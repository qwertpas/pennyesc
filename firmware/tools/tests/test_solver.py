import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from pennycal import (  # noqa: E402
    CAL_CAPTURE_POINTS_PER_DIR,
    CAL_CAPTURE_TOTAL_POINTS,
    CAL_POINTS_PER_SWEEP,
    CAL_ROTATIONS_PER_DIR,
    CAL_TOTAL_POINTS,
    CapturePoint,
    CalibrationError,
    LUT_SIZE,
    circular_mean_turn16,
    commutation_step_center_turn16,
    pseudo_index,
    reduce_capture_points,
    solve_capture,
    solve_legacy_csv,
)


class SolverTests(unittest.TestCase):
    def test_solver_replays_filtered_csvs(self) -> None:
        data_dir = Path(__file__).resolve().parents[2] / "pennyesc_libopencm3" / "data"
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
            points.append(CapturePoint(step, step, 0, x, y, 0, 0, 0, 0))

        for step in range(CAL_POINTS_PER_SWEEP):
            angle = 2.0 * math.pi * step / CAL_POINTS_PER_SWEEP + math.radians(15.0)
            x = int(round(radius * math.cos(angle)))
            y = int(round(radius * math.sin(angle)))
            points.append(CapturePoint(CAL_POINTS_PER_SWEEP + step, step, 1, x, y, 0, 0, 0, 0))

        with self.assertRaises(CalibrationError):
            solve_capture(points)

    def test_reduce_capture_preserves_step_labels(self) -> None:
        points = []
        for sweep_dir in (0, 1):
            base = 0 if sweep_dir == 0 else CAL_CAPTURE_POINTS_PER_DIR
            for rotation in range(CAL_ROTATIONS_PER_DIR):
                for local in range(CAL_POINTS_PER_SWEEP):
                    raw_index = base + rotation * CAL_POINTS_PER_SWEEP + local
                    step = local if sweep_dir == 0 else CAL_POINTS_PER_SWEEP - 1 - local
                    x = step * 100 + rotation
                    y = step * -100 - rotation
                    points.append(CapturePoint(raw_index, step, sweep_dir, x, y, 0, 0, 0, 0))

        self.assertEqual(len(points), CAL_CAPTURE_TOTAL_POINTS)
        reduced = reduce_capture_points(points)
        self.assertEqual(len(reduced), CAL_TOTAL_POINTS)
        for point in reduced:
            self.assertEqual(point.x, point.step_index * 100 + 1)
            self.assertEqual(point.y, point.step_index * -100 - 1)

    def test_reduce_capture_rejects_noisy_sample_window(self) -> None:
        points = []
        for sweep_dir in (0, 1):
            base = 0 if sweep_dir == 0 else CAL_CAPTURE_POINTS_PER_DIR
            for rotation in range(CAL_ROTATIONS_PER_DIR):
                for local in range(CAL_POINTS_PER_SWEEP):
                    raw_index = base + rotation * CAL_POINTS_PER_SWEEP + local
                    step = local if sweep_dir == 0 else CAL_POINTS_PER_SWEEP - 1 - local
                    spread = 20000 if raw_index == 0 else 0
                    points.append(CapturePoint(raw_index, step, sweep_dir, 0, 0, 0, 0, spread, 0))

        with self.assertRaisesRegex(CalibrationError, "capture sample spread too high"):
            reduce_capture_points(points)

    def test_pseudo_index_range(self) -> None:
        values = {pseudo_index(1000, 0), pseudo_index(0, 1000), pseudo_index(-1000, 0), pseudo_index(0, -1000)}
        self.assertTrue(all(0 <= value < LUT_SIZE for value in values))
        self.assertEqual(len(values), 4)

    def test_alignment_average_wraparound(self) -> None:
        mean = circular_mean_turn16((65530, 4, 8))
        self.assertTrue(mean < 16 or mean > 65520)

    def test_step_center_alignment(self) -> None:
        self.assertEqual(commutation_step_center_turn16(0), 5461)
        self.assertEqual(commutation_step_center_turn16(6), 5461)


if __name__ == "__main__":
    unittest.main()
