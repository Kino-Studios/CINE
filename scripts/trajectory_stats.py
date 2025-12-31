#!/usr/bin/env python3
"""Summarize baseline and cinematic trajectory files with descriptive statistics."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable


@dataclass(frozen=True)
class ColumnSummary:
    count: int
    mean: float
    std: float
    min: float
    max: float
    median: float


class ColumnAccumulator:
    """Keep running aggregates so we can emit summary statistics later."""

    def __init__(self) -> None:
        self.count = 0
        self.sum = 0.0
        self.sum_sq = 0.0
        self.min = math.inf
        self.max = -math.inf
        self.values: list[float] = []

    def add(self, value: float) -> None:
        self.count += 1
        self.sum += value
        self.sum_sq += value * value
        self.values.append(value)
        if value < self.min:
            self.min = value
        if value > self.max:
            self.max = value

    def summary(self) -> ColumnSummary:
        if self.count == 0:
            nan = float("nan")
            return ColumnSummary(count=0, mean=nan, std=nan, min=nan, max=nan, median=nan)
        mean = self.sum / self.count
        variance = max(self.sum_sq / self.count - mean * mean, 0.0)
        std = math.sqrt(variance)
        median = statistics.median(self.values)
        return ColumnSummary(
            count=self.count,
            mean=mean,
            std=std,
            min=self.min,
            max=self.max,
            median=median,
        )


def collect_statistics(files: Iterable[Path]) -> OrderedDict[str, ColumnSummary]:
    accumulators: "OrderedDict[str, ColumnAccumulator]" = OrderedDict()
    for csv_path in sorted(files):
        with csv_path.open(newline="") as handle:
            reader = csv.DictReader(handle)
            headers = reader.fieldnames or []
            for header in headers:
                if header not in accumulators:
                    accumulators[header] = ColumnAccumulator()
            for row in reader:
                for header in headers:
                    raw = row.get(header)
                    if raw is None:
                        continue
                    value = raw.strip()
                    if not value:
                        continue
                    try:
                        numeric = float(value)
                    except ValueError:
                        continue
                    accumulators[header].add(numeric)
    return OrderedDict((name, accumulator.summary()) for name, accumulator in accumulators.items())


def format_float(value: float) -> str:
    if math.isnan(value):
        return "nan"
    return f"{value:.6e}"


def emit_csv(output_path: Path, summary: "OrderedDict[str, ColumnSummary]") -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["column", "count", "mean", "std", "min", "max", "median"])
        for column, stats in summary.items():
            writer.writerow(
                [
                    column,
                    str(stats.count),
                    format_float(stats.mean),
                    format_float(stats.std),
                    format_float(stats.min),
                    format_float(stats.max),
                    format_float(stats.median),
                ]
            )


def summarize_dataset(dataset_path: Path) -> OrderedDict[str, ColumnSummary]:
    csv_files = sorted(dataset_path.glob("*.csv"))
    if not csv_files:
        raise FileNotFoundError(f"No CSV files found inside {dataset_path}")
    return collect_statistics(csv_files)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compute dataset-level statistics for each trajectory CSV directory."
    )
    parser.add_argument(
        "--trajectory-root",
        type=Path,
        default=Path("trajectory_data"),
        help="Root folder containing the dataset directories.",
    )
    parser.add_argument(
        "--datasets",
        nargs="+",
        default=["baseline", "cinematic"],
        help="Subdirectories under the root to summarize.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Where to write the summary CSVs (defaults to the trajectory root).",
    )
    args = parser.parse_args()

    root = args.trajectory_root
    output_dir = args.output_dir or root
    status = 0
    for dataset in args.datasets:
        dataset_path = root / dataset
        if not dataset_path.is_dir():
            print(f"Skipping {dataset}: {dataset_path} is not a directory", file=sys.stderr)
            status = 1
            continue
        try:
            summary = summarize_dataset(dataset_path)
        except FileNotFoundError as error:
            print(f"{error}", file=sys.stderr)
            status = 1
            continue
        output_path = output_dir / f"{dataset}_stats.csv"
        emit_csv(output_path, summary)
        print(f"Wrote summary for {dataset} to {output_path}")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
