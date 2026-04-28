from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt

from pnyproto import OBSERVER_NAMES


METRICS = (
    ("baseline_rpm", "baseline rpm", "baseline_rpm_vs_lead.png"),
    ("accel_0_50_rpm_s", "0-50 ms accel (rpm/s)", "accel_vs_lead.png"),
    ("end_rpm", "end rpm", "end_rpm_vs_lead.png"),
)


def finite(value: str) -> float:
    try:
        out = float(value)
    except ValueError:
        return math.nan
    return out if math.isfinite(out) else math.nan


def load_rows(path: Path) -> list[dict[str, object]]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    for row in rows:
        for key in (
            "direction",
            "observer_lead_us",
            "observer_mode",
            "pulse_duty",
            "sample_count",
            "missed_count",
            "mct_fault_delta",
            "faults",
        ):
            row[key] = int(row[key])
        for key in (
            "baseline_rpm",
            "accel_0_50_rpm_s",
            "end_rpm",
        ):
            row[key] = finite(row[key])
        row["valid"] = row["valid"] == "True"
    return rows


def mode_name(mode: int) -> str:
    return OBSERVER_NAMES.get(mode, f"mode{mode}")


def mean(values: list[float]) -> float:
    values = [v for v in values if math.isfinite(v)]
    return sum(values) / len(values) if values else math.nan


def write_scores(rows: list[dict[str, object]], out_dir: Path) -> list[dict[str, object]]:
    groups: dict[tuple[int, int, int], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        groups[(row["observer_mode"], row["observer_lead_us"], row["pulse_duty"])].append(row)

    scores: list[dict[str, object]] = []
    for (mode, lead, duty), group in sorted(groups.items()):
        valid = [row for row in group if row["valid"]]
        plus = [row for row in valid if row["direction"] == 1]
        minus = [row for row in valid if row["direction"] == -1]
        score = {
            "observer_mode": mode,
            "observer": mode_name(mode),
            "lead_us": lead,
            "duty": duty,
            "valid_count": len(valid),
            "total_count": len(group),
            "both_dirs": bool(plus and minus),
            "mean_baseline_rpm": mean([row["baseline_rpm"] for row in valid]),
            "mean_accel_0_50_rpm_s": mean([row["accel_0_50_rpm_s"] for row in valid]),
            "mean_end_rpm": mean([row["end_rpm"] for row in valid]),
            "fail_notes": ";".join(sorted({str(row["note"]) for row in group if not row["valid"] and row["note"]})),
        }
        scores.append(score)

    path = out_dir / "observer_scores.csv"
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(scores[0].keys()))
        writer.writeheader()
        writer.writerows(scores)
    return scores


def plot_metric(rows: list[dict[str, object]], metric: str, ylabel: str, path: Path) -> None:
    duties = sorted({row["pulse_duty"] for row in rows})
    dirs = [-1, 1]
    modes = sorted({row["observer_mode"] for row in rows})

    fig, axes = plt.subplots(len(duties), len(dirs), figsize=(12, 3.3 * len(duties)), sharex=True)
    if len(duties) == 1:
        axes = [axes]

    for duty_index, duty in enumerate(duties):
        for dir_index, direction in enumerate(dirs):
            ax = axes[duty_index][dir_index]
            for mode in modes:
                points = []
                for lead in sorted({row["observer_lead_us"] for row in rows}):
                    vals = [
                        row[metric]
                        for row in rows
                        if row["valid"]
                        and row["pulse_duty"] == duty
                        and row["direction"] == direction
                        and row["observer_mode"] == mode
                        and row["observer_lead_us"] == lead
                    ]
                    if vals:
                        points.append((lead, mean(vals)))
                if points:
                    xs, ys = zip(*points)
                    ax.plot(xs, ys, marker="o", linewidth=1.4, markersize=3, label=mode_name(mode))

            ax.set_title(f"duty {duty}, dir {direction:+d}")
            ax.set_xlabel("lead (us)")
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.25)
    handles, labels = axes[0][0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=min(6, len(labels)))
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(path, dpi=160)
    plt.close(fig)


def write_report(scores: list[dict[str, object]], out_dir: Path) -> None:
    usable = [
        score
        for score in scores
        if score["both_dirs"]
        and math.isfinite(score["mean_accel_0_50_rpm_s"])
    ]
    usable.sort(key=lambda row: row["mean_accel_0_50_rpm_s"], reverse=True)

    lines = ["# Observer Metric Summary", ""]
    lines.append("Top robust candidates require both spin directions valid for that duty.")
    lines.append("")
    lines.append("| observer | lead_us | duty | baseline_rpm | accel_0_50_rpm_s | end_rpm |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for row in usable[:20]:
        lines.append(
            f"| {row['observer']} | {row['lead_us']} | {row['duty']} | "
            f"{row['mean_baseline_rpm']:.1f} | {row['mean_accel_0_50_rpm_s']:.1f} | {row['mean_end_rpm']:.1f} |"
        )

    failures = defaultdict(int)
    totals = defaultdict(int)
    for row in scores:
        totals[row["observer"]] += row["total_count"]
        failures[row["observer"]] += row["total_count"] - row["valid_count"]
    lines.append("")
    lines.append("| observer | invalid / total |")
    lines.append("|---|---:|")
    for observer in sorted(totals):
        lines.append(f"| {observer} | {failures[observer]} / {totals[observer]} |")

    (out_dir / "observer_report.md").write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("summary", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()

    out_dir = args.out or args.summary.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = load_rows(args.summary)
    scores = write_scores(rows, out_dir)
    for metric, ylabel, filename in METRICS:
        plot_metric(rows, metric, ylabel, out_dir / filename)
    write_report(scores, out_dir)
    print(out_dir)


if __name__ == "__main__":
    main()
