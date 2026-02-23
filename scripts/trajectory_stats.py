#!/usr/bin/env python3
"""Summarize baseline and cinematic trajectory files with statistics."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import pandas as pd


@dataclass(frozen=True)
class ColumnSummary:
    count: int
    mean: float
    std: float
    min: float
    max: float
    median: float


def collect_statistics(
    files: Iterable[Path]
) -> OrderedDict[str, ColumnSummary]:
    dfs = []
    for csv_path in sorted(files):
        try:
            # Read CSV, let pandas infer types.
            # We will coerce to numeric later.
            df = pd.read_csv(csv_path)
            dfs.append(df)
        except pd.errors.EmptyDataError:
            # Matches original behavior where empty files are ignored
            pass

    if not dfs:
        return OrderedDict()

    # Concatenate all dataframes, preserving column order from files
    full_df = pd.concat(dfs, axis=0, ignore_index=True, sort=False)

    stats = OrderedDict()
    for col in full_df.columns:
        # Convert column to numeric, coercing errors to NaN
        # This mimics the try-except ValueError block in the original code
        numeric_series = pd.to_numeric(full_df[col], errors='coerce')

        # Drop NaNs (originally: if raw is None/empty or ValueError)
        numeric_series = numeric_series.dropna()

        count = len(numeric_series)
        if count == 0:
            nan = float("nan")
            stats[col] = ColumnSummary(
                count=0, mean=nan, std=nan, min=nan, max=nan, median=nan
            )
        else:
            # Calculate statistics
            # Original uses population variance (divide by N)
            std = numeric_series.std(ddof=0)
            mean = numeric_series.mean()
            min_val = float(numeric_series.min())
            max_val = float(numeric_series.max())
            median = float(numeric_series.median())

            stats[col] = ColumnSummary(
                count=count,
                mean=mean,
                std=std,
                min=min_val,
                max=max_val,
                median=median,
            )

    return stats


def format_float(value: float) -> str:
    if math.isnan(value):
        return "nan"
    return f"{value:.6e}"


def emit_csv(
    output_path: Path, summary: "OrderedDict[str, ColumnSummary]"
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ["column", "count", "mean", "std", "min", "max", "median"]
        )
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
        description=(
            "Compute dataset-level statistics for each "
            "trajectory CSV directory."
        )
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
        help=(
            "Where to write the summary CSVs (default: trajectory root)."
        ),
    )
    args = parser.parse_args()

    root = args.trajectory_root
    output_dir = args.output_dir or root
    status = 0
    for dataset in args.datasets:
        dataset_path = root / dataset
        if not dataset_path.is_dir():
            print(
                f"Skipping {dataset}: {dataset_path} is not a directory",
                file=sys.stderr
            )
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
