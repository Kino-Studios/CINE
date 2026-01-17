from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict, OrderedDict
from pathlib import Path
from typing import Sequence, Tuple

import numpy as np


def read_run(
    csv_path: Path, columns_order: OrderedDict[str, None]
) -> Tuple[dict[str, tuple[list[float], list[float]]], float, float] | None:
    column_points: dict[str, list[tuple[float, float]]] = defaultdict(list)
    min_time = math.inf
    max_time = -math.inf
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or "time" not in reader.fieldnames:
            return None
        for row in reader:
            raw_time = (row.get("time") or "").strip()
            if not raw_time:
                continue
            try:
                time_value = float(raw_time)
            except ValueError:
                continue
            min_time = min(min_time, time_value)
            max_time = max(max_time, time_value)
            for column, raw_value in row.items():
                if column == "time" or raw_value is None:
                    continue
                value_str = raw_value.strip()
                if not value_str:
                    continue
                try:
                    numeric = float(value_str)
                except ValueError:
                    continue
                column_points[column].append((time_value, numeric))
                if column not in columns_order:
                    columns_order[column] = None
    if min_time == math.inf:
        return None
    column_series: dict[str, tuple[list[float], list[float]]] = {}
    for column, points in column_points.items():
        points.sort(key=lambda pair: pair[0])
        times, values = zip(*points)
        column_series[column] = (list(times), list(values))
    return column_series, min_time, max_time


def load_runs(
    csv_paths: Sequence[Path],
) -> Tuple[list[dict[str, tuple[list[float], list[float]]]], OrderedDict[str, None], float, float]:
    runs: list[dict[str, tuple[list[float], list[float]]]] = []
    columns_order: OrderedDict[str, None] = OrderedDict()
    min_time = math.inf
    max_time = -math.inf
    for csv_path in sorted(csv_paths):
        result = read_run(csv_path, columns_order)
        if result is None:
            continue
        run_data, run_min, run_max = result
        runs.append(run_data)
        min_time = min(min_time, run_min)
        max_time = max(max_time, run_max)
    return runs, columns_order, min_time, max_time


def build_time_grid(min_time: float, max_time: float, step: float) -> list[float]:
    if min_time == math.inf or max_time == -math.inf:
        return []
    if max_time <= min_time:
        return [min_time]
    count = max(1, int(math.floor((max_time - min_time) / step)))
    grid = [min_time + i * step for i in range(count + 1)]
    if grid[-1] < max_time - 1e-12:
        grid.append(max_time)
    return grid


def resample_run(
    series: dict[str, tuple[list[float], list[float]]], grid: list[float]
) -> dict[str, list[float]]:
    resampled: dict[str, list[float]] = {}
    for column, (times, values) in series.items():
        if not times:
            continue
        if len(times) == 1:
            single_time = times[0]
            single_value = values[0]
            data = []
            for t in grid:
                if math.isclose(t, single_time, rel_tol=1e-9, abs_tol=1e-12):
                    data.append(single_value)
                else:
                    data.append(math.nan)
            resampled[column] = data
            continue
        interpolated = np.interp(grid, times, values, left=np.nan, right=np.nan)
        resampled[column] = list(interpolated)
    return resampled


def aggregate_runs(
    runs: list[dict[str, tuple[list[float], list[float]]]],
    grid: list[float],
    columns_order: OrderedDict[str, None],
) -> dict[str, list[list[float]]]:
    grid_len = len(grid)
    column_stats: dict[str, list[list[float]]] = {
        column: [[0.0, 0.0, 0] for _ in range(grid_len)] for column in columns_order
    }
    for run in runs:
        resampled = resample_run(run, grid)
        for column, values in resampled.items():
            stats = column_stats.setdefault(
                column, [[0.0, 0.0, 0] for _ in range(grid_len)]
            )
            for idx, value in enumerate(values):
                if math.isnan(value):
                    continue
                stats[idx][0] += value
                stats[idx][1] += value * value
                stats[idx][2] += 1
    return column_stats


def compute_mean(
    csv_files: Sequence[Path], time_step: float
) -> Tuple[list[float], OrderedDict[str, None], dict[str, list[list[float]]]]:
    if not csv_files:
        raise FileNotFoundError("No CSV files provided for averaging.")
    runs, columns_order, min_time, max_time = load_runs(csv_files)
    if not runs or not columns_order:
        raise FileNotFoundError("No numeric data found in provided CSVs.")
    grid = build_time_grid(min_time, max_time, time_step)
    if not grid:
        raise FileNotFoundError("Unable to construct a time grid from the provided data.")
    column_stats = aggregate_runs(runs, grid, columns_order)
    return grid, columns_order, column_stats


def write_mean_std_csv(
    output_path: Path,
    grid: list[float],
    columns_order: OrderedDict[str, None],
    column_stats: dict[str, list[list[float]]],
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    columns = ["time", *(f"{col}_mean" for col in columns_order), *(f"{col}_std" for col in columns_order)]
    with output_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(columns)
        for idx, time_value in enumerate(grid):
            row = [f"{time_value:.9f}"]
            mean_cells = []
            std_cells = []
            for column in columns_order:
                stats = column_stats.get(column)
                if stats is None or idx >= len(stats):
                    mean_cells.append("")
                    std_cells.append("")
                    continue
                sum_value, sum_sq, count = stats[idx]
                if count == 0:
                    mean_cells.append("")
                    std_cells.append("")
                    continue
                mean = sum_value / count
                variance = sum_sq / count - mean * mean
                if variance < 0.0:
                    variance = 0.0
                mean_cells.append(f"{mean:.9e}")
                std_cells.append(f"{math.sqrt(variance):.9e}")
            row.extend(mean_cells)
            row.extend(std_cells)
            writer.writerow(row)


def process_dataset(
    name: str,
    csv_paths: Sequence[Path],
    output_path: Path,
    time_step: float,
) -> Path:
    grid, columns_order, column_stats = compute_mean(csv_paths, time_step)
    write_mean_std_csv(output_path, grid, columns_order, column_stats)
    return output_path


def discover_csvs(root: Path, dataset: str) -> list[Path]:
    dataset_path = root / dataset
    if not dataset_path.is_dir():
        return []
    return sorted(dataset_path.glob("*.csv"))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Resample trajectories onto a uniform time base and average the runs."
    )
    parser.add_argument(
        "--trajectory-root",
        type=Path,
        default=Path("trajectory_data"),
        help="Root directory containing the dataset folders.",
    )
    parser.add_argument(
        "--datasets",
        nargs="+",
        default=["baseline", "cinematic"],
        help="Subdirectories under the root to process.",
    )
    parser.add_argument(
        "--time-step",
        type=float,
        default=0.005,
        help="Resampling interval (seconds) for the uniform time grid.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory where dataset mean files are written (defaults to the trajectory root).",
    )
    parser.add_argument(
        "--combined-output",
        type=Path,
        default=None,
        help="Optional path for a merged mean trajectory combining every run.",
    )
    args = parser.parse_args()

    root = args.trajectory_root
    output_dir = args.output_dir or root
    combined_candidates: list[Path] = []
    exit_status = 0
    for dataset in args.datasets:
        csv_paths = discover_csvs(root, dataset)
        if not csv_paths:
            print(f"Skipping {dataset}: no CSVs found in {root / dataset}")
            exit_status = 1
            continue
        combined_candidates.extend(csv_paths)
        output_path = output_dir / f"{dataset}_mean_std_trajectory.csv"
        try:
            written_path = process_dataset(
                dataset, csv_paths, output_path, args.time_step
            )
        except FileNotFoundError as exc:
            print(f"Error for {dataset}: {exc}")
            exit_status = 1
            continue
        print(f"Wrote time-aligned mean+std trajectory for {dataset} to {written_path}")

    if args.combined_output:
        if not combined_candidates:
            print("No CSVs available to build combined mean trajectory", file=sys.stderr)
            exit_status = 1
        else:
            combined_path = args.combined_output
            try:
                process_dataset(
                    "combined",
                    combined_candidates,
                    combined_path,
                    args.time_step,
                )
            except FileNotFoundError as exc:
                print(f"Error building combined trajectory: {exc}", file=sys.stderr)
                exit_status = 1
            else:
                print(f"Wrote combined time-aligned mean+std trajectory to {combined_path}")
    return exit_status


if __name__ == "__main__":
    raise SystemExit(main())
