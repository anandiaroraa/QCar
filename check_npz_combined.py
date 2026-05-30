#!/usr/bin/env python3
"""Analyze the latest combined QCar run with both cars and the pushed block."""

import argparse
import glob
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


DEFAULT_PATTERN = "hardware_results_combined_test1/*.npz"


def _load_history(data, key, min_columns, required=True):
    if key not in data:
        if not required:
            return None
        raise KeyError(f"Missing required array: {key}")
    history = np.asarray(data[key], dtype=float)
    if history.size == 0 and not required:
        return None
    if history.ndim != 2 or history.shape[1] < min_columns:
        raise ValueError(f"{key} must be a 2D array with at least {min_columns} columns")
    return history


def _load_reference(data, prefix, fallback_prefix=None):
    x_key = f"{prefix}_reference_x"
    y_key = f"{prefix}_reference_y"
    if x_key not in data or y_key not in data:
        if fallback_prefix is None:
            return None
        x_key = f"{fallback_prefix}_x"
        y_key = f"{fallback_prefix}_y"
        if x_key not in data or y_key not in data:
            return None
    ref_x = np.asarray(data[x_key], dtype=float).reshape(-1)
    ref_y = np.asarray(data[y_key], dtype=float).reshape(-1)
    if ref_x.size < 2 or ref_y.size != ref_x.size:
        return None
    return np.column_stack((ref_x, ref_y))


def _path_error(points, reference):
    """Return distance from each measured point to the saved path segments."""
    starts = reference[:-1]
    segments = reference[1:] - starts
    segment_length_sq = np.sum(segments * segments, axis=1)
    valid = segment_length_sq > 1e-12
    if not np.any(valid):
        return np.linalg.norm(points - reference[0], axis=1)
    starts = starts[valid]
    segments = segments[valid]
    segment_length_sq = segment_length_sq[valid]

    offsets = points[:, None, :] - starts[None, :, :]
    fractions = np.sum(offsets * segments[None, :, :], axis=2) / segment_length_sq[None, :]
    fractions = np.clip(fractions, 0.0, 1.0)
    projections = starts[None, :, :] + fractions[:, :, None] * segments[None, :, :]
    distances = np.linalg.norm(points[:, None, :] - projections, axis=2)
    return np.min(distances, axis=1)


def _metrics(errors):
    return {
        "mean": float(np.mean(errors)),
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "std": float(np.std(errors)),
        "max": float(np.max(errors)),
    }


def _position_summary(history):
    start = history[0, :2]
    final = history[-1, :2]
    return {
        "start": start,
        "final": final,
        "displacement": float(np.linalg.norm(final - start)),
    }


def _separation(first, second, sample_index):
    return float(np.linalg.norm(first[sample_index, :2] - second[sample_index, :2]))


def _latest_path(pattern):
    files = sorted(glob.glob(pattern), key=os.path.getmtime)
    if not files:
        raise FileNotFoundError(f"No .npz files found for pattern: {pattern}")
    return Path(files[-1])


def _plot_entity(ax, history, reference, label, color):
    if reference is not None:
        ax.plot(reference[:, 0], reference[:, 1], "--", color=color, alpha=0.7,
                label=f"{label} reference")
    ax.plot(history[:, 0], history[:, 1], "-", color=color, linewidth=1.8,
            label=f"{label} measured")
    ax.plot(history[0, 0], history[0, 1], "o", color=color)


def analyze(path, output=None, show=False):
    data = np.load(path, allow_pickle=True)
    car1 = _load_history(data, "car1_history", 7)
    car2 = _load_history(data, "car2_history", 7)
    block = _load_history(data, "block_history", 4, required=False)

    entities = [
        ("QCar1", car1, _load_reference(data, "car1", "reference"), "tab:blue", 6),
        ("QCar2", car2, _load_reference(data, "car2"), "tab:purple", 6),
        ("Block", block, _load_reference(data, "block"), "tab:green", 3),
    ]
    results = {}
    for label, history, reference, _, _ in entities:
        if history is not None and reference is not None:
            results[label] = _path_error(history[:, :2], reference)

    print(f"Loading: {path}")
    print(f"Samples: QCar1={len(car1)}, QCar2={len(car2)}, Block={len(block) if block is not None else 0}")
    print("\nAverage tracking errors (distance to saved reference path):")
    print(f"{'Entity':<8} {'Mean [m]':>10} {'RMSE [m]':>10} {'Std [m]':>10} {'Max [m]':>10}")
    for label, history, reference, _, _ in entities:
        if history is None:
            print(f"{label:<8} {'N/A (no recorded poses)':>43}")
            continue
        if reference is None:
            print(f"{label:<8} {'N/A (no saved reference)':>43}")
            continue
        stats = _metrics(results[label])
        print(
            f"{label:<8} {stats['mean']:>10.4f} {stats['rmse']:>10.4f} "
            f"{stats['std']:>10.4f} {stats['max']:>10.4f}"
        )

    print("\nMeasured initial and final positions:")
    print(f"{'Entity':<8} {'Initial (x, y) [m]':>24} {'Final (x, y) [m]':>24} {'Moved [m]':>11}")
    for label, history, reference, _, _ in entities:
        if history is None:
            print(f"{label:<8} {'N/A (no recorded poses)':>61}")
            continue
        summary = _position_summary(history)
        start = summary["start"]
        final = summary["final"]
        print(
            f"{label:<8} ({start[0]:>8.4f}, {start[1]:>8.4f}) "
            f"({final[0]:>8.4f}, {final[1]:>8.4f}) {summary['displacement']:>11.4f}"
        )

    # print("\nInitial and final separations between recorded objects:")
    # print(f"{'Objects':<16} {'Initial [m]':>12} {'Final [m]':>12} {'Change [m]':>12}")
    # pairs = [("QCar1-Block", car1, block), ("QCar2-Block", car2, block), ("QCar1-QCar2", car1, car2)]
    # for label, first, second in pairs:
    #     if first is None or second is None:
    #         print(f"{label:<16} {'N/A (no recorded block poses)':>38}")
    #         continue
    #     initial = _separation(first, second, 0)
    #     final = _separation(first, second, -1)
    #     print(f"{label:<16} {initial:>12.4f} {final:>12.4f} {final - initial:>12.4f}")

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    for label, history, reference, color, _ in entities:
        if history is None:
            continue
        _plot_entity(axes[0], history, reference, label, color)
    axes[0].set_title("Measured Poses And Saved References")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].axis("equal")
    axes[0].grid(True)
    axes[0].legend(fontsize=8)

    for label, history, _, color, timestamp_col in entities:
        if history is None or label not in results:
            continue
        t = history[:, timestamp_col] - history[0, timestamp_col]
        axes[1].plot(t, results[label], color=color, label=label)
    axes[1].set_title("Tracking Error Over Time")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("distance to reference [m]")
    axes[1].grid(True)
    axes[1].legend()
    fig.tight_layout()

    if output is None:
        output = Path("Tuning_straight") / f"{path.stem}_combined_errors.png"
    else:
        output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160, bbox_inches="tight")
    print(f"\nSaved plot to {output}")
    if show:
        plt.show()
    else:
        plt.close(fig)
    return results


def main():
    parser = argparse.ArgumentParser(
        description="Report path tracking errors for QCar1, QCar2, and the block."
    )
    parser.add_argument("npz", nargs="?", type=Path,
                        help="Combined run .npz file; defaults to the latest combined test run.")
    parser.add_argument("--pattern", default=DEFAULT_PATTERN,
                        help="Glob pattern used when no .npz path is supplied.")
    parser.add_argument("--output", help="Output PNG path.")
    parser.add_argument("--show", action="store_true", help="Show the generated figure.")
    args = parser.parse_args()

    path = args.npz if args.npz is not None else _latest_path(args.pattern)
    analyze(path, output=args.output, show=args.show)


if __name__ == "__main__":
    main()
