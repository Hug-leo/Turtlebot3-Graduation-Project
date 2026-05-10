#!/usr/bin/env python3
"""Export report-ready DRL training plots from metrics.csv/stage_summary.csv.

Example:
    python3 tools/export_training_plots.py \
      --run-dir /home/hug/tb3_sim_ws/runs/gap_sac_20260506_230839
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt


DEFAULT_RUN_DIR = Path("/home/hug/tb3_sim_ws/runs/gap_sac_20260506_230839")


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        raise FileNotFoundError(f"CSV file not found: {path}")
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _to_float(value: str | None, default: float = math.nan) -> float:
    if value is None or value == "":
        return default
    if value == "True":
        return 1.0
    if value == "False":
        return 0.0
    try:
        return float(value)
    except ValueError:
        return default


def _series(rows: Iterable[dict[str, str]], key: str) -> list[float]:
    return [_to_float(row.get(key)) for row in rows]


def _moving_average(values: list[float], window: int) -> list[float]:
    if window <= 1:
        return values
    averaged: list[float] = []
    for index in range(len(values)):
        start = max(0, index - window + 1)
        chunk = [v for v in values[start : index + 1] if not math.isnan(v)]
        averaged.append(sum(chunk) / len(chunk) if chunk else math.nan)
    return averaged


def _set_report_style() -> None:
    plt.rcParams.update(
        {
            "figure.figsize": (10, 5.8),
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 11,
            "axes.grid": True,
            "grid.alpha": 0.28,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.frameon": False,
        }
    )


def _save(fig: plt.Figure, output_path: Path, formats: list[str]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    for fmt in formats:
        fig.savefig(output_path.with_suffix(f".{fmt}"), bbox_inches="tight")
    plt.close(fig)


def plot_reward_and_success(
    rows: list[dict[str, str]],
    output_dir: Path,
    rolling_window: int,
    formats: list[str],
) -> None:
    episodes = [int(_to_float(row.get("episode"), index)) for index, row in enumerate(rows)]
    rewards = _series(rows, "episode_reward")
    rolling_reward = _moving_average(rewards, rolling_window)

    success = _series(rows, "success")
    rolling_success = _series(rows, "rolling_success_rate")
    if all(math.isnan(v) for v in rolling_success):
        rolling_success = _moving_average(success, rolling_window)

    fig, reward_axis = plt.subplots()
    success_axis = reward_axis.twinx()

    reward_axis.plot(episodes, rewards, color="#b7b7b7", linewidth=0.9, alpha=0.55, label="Episode reward")
    reward_axis.plot(
        episodes,
        rolling_reward,
        color="#145DA0",
        linewidth=2.2,
        label=f"Reward moving avg ({rolling_window})",
    )
    success_axis.plot(
        episodes,
        rolling_success,
        color="#1B998B",
        linewidth=2.2,
        label="Rolling success rate",
    )

    reward_axis.set_title("DRL Training Reward and Success Rate")
    reward_axis.set_xlabel("Episode")
    reward_axis.set_ylabel("Reward")
    success_axis.set_ylabel("Success rate")
    success_axis.set_ylim(-0.05, 1.05)

    lines_a, labels_a = reward_axis.get_legend_handles_labels()
    lines_b, labels_b = success_axis.get_legend_handles_labels()
    reward_axis.legend(lines_a + lines_b, labels_a + labels_b, loc="lower right")

    _save(fig, output_dir / "training_reward_success", formats)


def plot_distance_and_clearance(
    rows: list[dict[str, str]],
    output_dir: Path,
    rolling_window: int,
    formats: list[str],
) -> None:
    episodes = [int(_to_float(row.get("episode"), index)) for index, row in enumerate(rows)]
    final_distance = _moving_average(_series(rows, "final_DG"), rolling_window)
    min_obstacle = _moving_average(_series(rows, "min_obstacle_distance"), rolling_window)

    fig, axis = plt.subplots()
    axis.plot(episodes, final_distance, color="#D1495B", linewidth=2.2, label="Final distance to goal")
    axis.plot(episodes, min_obstacle, color="#30638E", linewidth=2.2, label="Minimum obstacle distance")
    axis.set_title("Goal Accuracy and Obstacle Clearance")
    axis.set_xlabel("Episode")
    axis.set_ylabel("Distance (m)")
    axis.legend(loc="best")

    _save(fig, output_dir / "training_distance_clearance", formats)


def plot_stage_summary(
    rows: list[dict[str, str]],
    output_dir: Path,
    formats: list[str],
) -> None:
    if not rows:
        return

    stage_names = [row.get("stage_name", f"stage_{index}") for index, row in enumerate(rows)]
    success_rates = [_to_float(row.get("success_rate"), 0.0) for row in rows]
    collision_rates = [_to_float(row.get("collision_rate"), 0.0) for row in rows]
    avg_final_distance = [_to_float(row.get("avg_final_DG"), 0.0) for row in rows]

    fig, axis = plt.subplots(figsize=(max(9, len(stage_names) * 1.35), 5.8))
    x = list(range(len(stage_names)))
    axis.bar([v - 0.18 for v in x], success_rates, width=0.36, color="#1B998B", label="Success rate")
    axis.bar([v + 0.18 for v in x], collision_rates, width=0.36, color="#D1495B", label="Collision rate")
    axis.set_ylim(0.0, 1.05)
    axis.set_ylabel("Rate")
    axis.set_title("Curriculum Stage Summary")
    axis.set_xticks(x)
    axis.set_xticklabels(stage_names, rotation=25, ha="right")

    distance_axis = axis.twinx()
    distance_axis.plot(x, avg_final_distance, color="#145DA0", marker="o", linewidth=2.0, label="Avg final distance")
    distance_axis.set_ylabel("Avg final distance (m)")

    lines_a, labels_a = axis.get_legend_handles_labels()
    lines_b, labels_b = distance_axis.get_legend_handles_labels()
    axis.legend(lines_a + lines_b, labels_a + labels_b, loc="upper right")

    _save(fig, output_dir / "stage_summary", formats)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=DEFAULT_RUN_DIR,
        help=f"Run directory containing metrics.csv and stage_summary.csv. Default: {DEFAULT_RUN_DIR}",
    )
    parser.add_argument("--metrics", type=Path, default=None, help="Optional explicit metrics.csv path.")
    parser.add_argument("--stage-summary", type=Path, default=None, help="Optional explicit stage_summary.csv path.")
    parser.add_argument("--output-dir", type=Path, default=None, help="Output directory for figures.")
    parser.add_argument("--rolling-window", type=int, default=20, help="Moving average window in episodes.")
    parser.add_argument(
        "--formats",
        nargs="+",
        default=["png", "pdf"],
        choices=["png", "pdf", "svg"],
        help="Figure output formats.",
    )
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    metrics_path = (args.metrics or run_dir / "metrics.csv").expanduser().resolve()
    stage_summary_path = (args.stage_summary or run_dir / "stage_summary.csv").expanduser().resolve()
    output_dir = (args.output_dir or run_dir / "figures").expanduser().resolve()

    _set_report_style()
    metrics_rows = _read_csv(metrics_path)
    stage_rows = _read_csv(stage_summary_path) if stage_summary_path.exists() else []

    plot_reward_and_success(metrics_rows, output_dir, args.rolling_window, args.formats)
    plot_distance_and_clearance(metrics_rows, output_dir, args.rolling_window, args.formats)
    plot_stage_summary(stage_rows, output_dir, args.formats)

    print(f"Exported figures to: {output_dir}")
    for path in sorted(output_dir.iterdir()):
        if path.suffix.lstrip(".") in set(args.formats):
            print(f"- {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
