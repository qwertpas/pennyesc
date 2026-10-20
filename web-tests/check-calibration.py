"""Verify browser fixtures with the authoritative Python calibration solver."""
import dataclasses
import json
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / "firmware/tools"))
from pennycal import CapturePoint, reduce_capture_points, solve_capture

fixture = json.loads((Path(__file__).parent / "calibration-fixture.json").read_text())
for entry in fixture["cases"]:
    raw = [CapturePoint(**dict(zip(fixture["fields"], row))) for row in entry["raw"]]
    reduced = reduce_capture_points(raw)
    assert [list(dataclasses.astuple(point)) for point in reduced] == entry["reduced"], entry["name"]
    solved = dataclasses.asdict(solve_capture(reduced))
    solved["blob"] = solved["blob"].hex()
    # JSON converts Python tuples to arrays.
    assert json.loads(json.dumps(solved)) == entry["solved"], entry["name"]
    print(f"{entry['name']}: Python solver matches fixture")
