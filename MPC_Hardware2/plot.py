import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


RUNS_BY_RADIUS = {
    1.8: [
        "hardware_results_test1/run_000_t_1777925686.58893.npz",
        "hardware_results_test1/run_000_t_1777926155.66282.npz",
        "hardware_results_test1/run_000_t_1777926363.21419.npz",
    ],
    2.0: [
        "hardware_results_test1/run_000_t_1777927290.61066.npz",
        "hardware_results_test1/run_000_t_1777927440.39650.npz",
        "hardware_results_test1/run_000_t_1777927602.61664.npz",
    ],
    2.2: [
        "hardware_results_test1/run_000_t_1777930286.04797.npz",
        "hardware_results_test1/run_000_t_1777932410.74605.npz",
        "hardware_results_test1/run_000_t_1777933691.84985.npz",
    ],
}

COLORS = {
    1.8: "tab:blue",
    2.0: "tab:orange",
    2.2: "tab:green",
}


def resample_by_progress(x, y, n_points=250):
    """Resample a trajectory by normalized arc-length progress."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    valid = ~(np.isnan(x) | np.isnan(y))
    x = x[valid]
    y = y[valid]

    if len(x) < 2:
        return np.full(n_points, np.nan), np.full(n_points, np.nan)

    ds = np.hypot(np.diff(x), np.diff(y))
    s = np.concatenate(([0.0], np.cumsum(ds)))
    keep = np.concatenate(([True], np.diff(s) > 1e-9))
    x = x[keep]
    y = y[keep]
    s = s[keep]

    if s[-1] <= 1e-9:
        return np.full(n_points, x[0]), np.full(n_points, y[0])

    progress = s / s[-1]
    target_progress = np.linspace(0.0, 1.0, n_points)
    return (
        np.interp(target_progress, progress, x),
        np.interp(target_progress, progress, y),
    )


def smooth_line(values, window=9):
    """Small moving-average smoother for cleaner mean/std lines."""
    values = np.asarray(values, dtype=float)
    if window <= 1 or len(values) < window:
        return values

    pad = window // 2
    padded = np.pad(values, pad, mode="edge")
    kernel = np.ones(window) / window
    return np.convolve(padded, kernel, mode="valid")


def load_group(paths, n_points=250):
    actual_x = []
    actual_y = []

    for path in paths:
        data = np.load(path, allow_pickle=True)
        hist = np.asarray(data["car1_history"])
        x, y = resample_by_progress(hist[:, 0], hist[:, 1], n_points=n_points)
        actual_x.append(x)
        actual_y.append(y)

    first = np.load(paths[0], allow_pickle=True)
    ref_x, ref_y = resample_by_progress(
        np.asarray(first["reference_x"]).reshape(-1),
        np.asarray(first["reference_y"]).reshape(-1),
        n_points=n_points,
    )

    actual_x = np.vstack(actual_x)
    actual_y = np.vstack(actual_y)
    mean_x = smooth_line(np.nanmean(actual_x, axis=0))
    mean_y = smooth_line(np.nanmean(actual_y, axis=0))
    std_x = smooth_line(np.nanstd(actual_x, axis=0))
    std_y = smooth_line(np.nanstd(actual_y, axis=0))

    # Use a combined XY std for a compact uncertainty band around the mean curve.
    std_xy = np.sqrt(std_x**2 + std_y**2)

    return mean_x, mean_y, std_xy, smooth_line(ref_x), smooth_line(ref_y)


def plot_std_band(ax, x, y, std_xy, color, label):
    dx = np.gradient(x)
    dy = np.gradient(y)
    length = np.hypot(dx, dy)
    length[length < 1e-9] = 1.0

    nx = -dy / length
    ny = dx / length
    upper_x = x + nx * std_xy
    upper_y = y + ny * std_xy
    lower_x = x - nx * std_xy
    lower_y = y - ny * std_xy

    band_x = np.concatenate([upper_x, lower_x[::-1]])
    band_y = np.concatenate([upper_y, lower_y[::-1]])
    ax.fill(
        band_x,
        band_y,
        color=color,
        alpha=0.12,
        linewidth=0,
        label=label,
    )


def apply_symmetric_limits(ax, limits):
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(*limits)
    ax.set_ylim(*limits)


def plot_lemniscate_radius_summary(output_dir="Tuning_lemniscate", n_points=250, limits=None):
    os.makedirs(output_dir, exist_ok=True)

    fig, ax = plt.subplots(figsize=(8, 8))

    for radius, filenames in RUNS_BY_RADIUS.items():
        paths = [Path(name) for name in filenames]
        missing = [path for path in paths if not path.exists()]
        if missing:
            raise FileNotFoundError(
                "Missing files for radius "
                f"{radius}: " + ", ".join(str(path) for path in missing)
            )

        mean_x, mean_y, std_xy, ref_x, ref_y = load_group(paths, n_points=n_points)
        color = COLORS[radius]

        ax.plot(
            mean_x,
            mean_y,
            "-",
            color=color,
            linewidth=2.5,
            label=f"Radius of curvature {radius:.1f} m mean actual",
        )
        plot_std_band(
            ax,
            mean_x,
            mean_y,
            std_xy,
            color,
            label=f"Radius of curvature {radius:.1f} m actual std dev",
        )
        ax.plot(
            ref_x,
            ref_y,
            ":",
            color=color,
            linewidth=2.5,
            label=f"Radius of curvature {radius:.1f} m reference",
        )

    ax.set_title("Mean Lemniscate Trajectory Tracking by Radius of Curvature")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if limits is None:
        limits = collect_all_limits(n_points=n_points)
    apply_symmetric_limits(ax, limits)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    output_path = Path(output_dir) / "combined_lemniscate_mean_std_reference.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    print(os.path.abspath(output_path))


def collect_all_limits(n_points=250, padding=0.25):
    xs = []
    ys = []

    for radius, filenames in RUNS_BY_RADIUS.items():
        paths = [Path(name) for name in filenames]
        mean_x, mean_y, std_xy, ref_x, ref_y = load_group(paths, n_points=n_points)
        xs.extend([mean_x - std_xy, mean_x + std_xy, ref_x])
        ys.extend([mean_y - std_xy, mean_y + std_xy, ref_y])

    all_x = np.concatenate(xs)
    all_y = np.concatenate(ys)
    max_abs = float(np.nanmax(np.abs(np.concatenate([all_x, all_y]))) + padding)
    return (-max_abs, max_abs)


def plot_single_radius(radius, output_dir="Tuning_lemniscate", n_points=250, limits=None):
    os.makedirs(output_dir, exist_ok=True)

    if radius not in RUNS_BY_RADIUS:
        raise ValueError(f"Unknown radius {radius}. Options: {sorted(RUNS_BY_RADIUS)}")

    paths = [Path(name) for name in RUNS_BY_RADIUS[radius]]
    missing = [path for path in paths if not path.exists()]
    if missing:
        raise FileNotFoundError(
            "Missing files for radius "
            f"{radius}: " + ", ".join(str(path) for path in missing)
        )

    mean_x, mean_y, std_xy, ref_x, ref_y = load_group(paths, n_points=n_points)
    color = COLORS[radius]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(
        mean_x,
        mean_y,
        "-",
        color=color,
        linewidth=2.5,
        label=f"Radius of curvature {radius:.1f} m mean actual",
    )
    plot_std_band(
        ax,
        mean_x,
        mean_y,
        std_xy,
        color,
        label=f"Radius of curvature {radius:.1f} m actual std dev",
    )
    ax.plot(
        ref_x,
        ref_y,
        ":",
        color=color,
        linewidth=2.5,
        label=f"Radius of curvature {radius:.1f} m reference",
    )

    ax.set_title(f"Lemniscate Trajectory Tracking, Radius of Curvature {radius:.1f} m")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if limits is not None:
        apply_symmetric_limits(ax, limits)
    else:
        ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)

    output_path = Path(output_dir) / f"lemniscate_{radius:.1f}_mean_std_reference.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    plt.close(fig)
    print(os.path.abspath(output_path))


def plot_each_radius(output_dir="Tuning_lemniscate", n_points=250):
    limits = collect_all_limits(n_points=n_points)
    for radius in RUNS_BY_RADIUS:
        plot_single_radius(radius, output_dir=output_dir, n_points=n_points, limits=limits)


if __name__ == "__main__":
    shared_limits = collect_all_limits()
    plot_lemniscate_radius_summary(limits=shared_limits)
    plot_each_radius()
