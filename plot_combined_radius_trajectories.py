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
        "hardware_results_test1/run_000_t_1777931858.76344.npz",
        "hardware_results_test1/run_000_t_1777933691.84985.npz",
    ],
}

COLORS = {
    1.8: "tab:blue",
    2.0: "tab:orange",
    2.2: "tab:green",
}

TABLE_METRICS = {
    1.8: {
        "rmse": [0.108, 0.112, 0.107],
        "mean": [0.080, 0.081, 0.083],
        "std": [0.072, 0.077, 0.068],
    },
    2.0: {
        "rmse": [0.079, 0.068, 0.074],
        "mean": [0.058, 0.050, 0.055],
        "std": [0.053, 0.046, 0.050],
    },
    2.2: {
        "rmse": [0.063, 0.072, 0.068],
        "mean": [0.044, 0.049, 0.048],
        "std": [0.045, 0.053, 0.048],
    },
}


def nearest_reference_indices(x, y, ref_x, ref_y):
    indices = []
    for start in range(0, len(x), 1000):
        end = start + 1000
        dx = x[start:end, None] - ref_x[None, :]
        dy = y[start:end, None] - ref_y[None, :]
        indices.append(np.argmin(dx * dx + dy * dy, axis=1))
    return np.concatenate(indices)


def tracking_error(x, y, ref_x, ref_y):
    errors = []
    for start in range(0, len(x), 1000):
        end = start + 1000
        dx = x[start:end, None] - ref_x[None, :]
        dy = y[start:end, None] - ref_y[None, :]
        errors.append(np.sqrt(np.min(dx * dx + dy * dy, axis=1)))
    return np.concatenate(errors)


def mean_actual_trajectory(paths, ref_x, ref_y):
    xs_by_ref = [[] for _ in ref_x]
    ys_by_ref = [[] for _ in ref_y]

    for path in paths:
        data = np.load(path, allow_pickle=True)
        hist = np.asarray(data["car1_history"])
        x = hist[:, 0]
        y = hist[:, 1]
        ref_indices = nearest_reference_indices(x, y, ref_x, ref_y)

        for xi, yi, ref_i in zip(x, y, ref_indices):
            xs_by_ref[ref_i].append(xi)
            ys_by_ref[ref_i].append(yi)

    mean_x = np.full_like(ref_x, np.nan, dtype=float)
    mean_y = np.full_like(ref_y, np.nan, dtype=float)

    for i, (xs, ys) in enumerate(zip(xs_by_ref, ys_by_ref)):
        if xs:
            mean_x[i] = np.mean(xs)
            mean_y[i] = np.mean(ys)

    valid = ~(np.isnan(mean_x) | np.isnan(mean_y))
    return mean_x[valid], mean_y[valid]


def metrics_for_paths(paths):
    rmses = []
    means = []
    stds = []

    for path in paths:
        data = np.load(path, allow_pickle=True)
        hist = np.asarray(data["car1_history"])
        x = hist[:, 0]
        y = hist[:, 1]
        ref_x = np.asarray(data["reference_x"]).reshape(-1)
        ref_y = np.asarray(data["reference_y"]).reshape(-1)
        errors = tracking_error(x, y, ref_x, ref_y)
        rmses.append(float(np.sqrt(np.mean(errors ** 2))))
        means.append(float(np.mean(errors)))
        stds.append(float(np.std(errors)))

    return np.array(rmses), np.array(means), np.array(stds)


def main():
    fig, ax = plt.subplots(figsize=(8, 8))
    summary_rows = []

    for radius, files in RUNS_BY_RADIUS.items():
        paths = [Path(file) for file in files]
        data = np.load(paths[0], allow_pickle=True)
        ref_x = np.asarray(data["reference_x"]).reshape(-1)
        ref_y = np.asarray(data["reference_y"]).reshape(-1)
        mean_x, mean_y = mean_actual_trajectory(paths, ref_x, ref_y)
        # rmses, means, stds = metrics_for_paths(paths)
        rmses = np.array(TABLE_METRICS[radius]["rmse"])
        means = np.array(TABLE_METRICS[radius]["mean"])
        stds = np.array(TABLE_METRICS[radius]["std"])
        color = COLORS[radius]

        ax.plot(
            mean_x,
            mean_y,
            "-",
            color=color,
            linewidth=2.2,
            label=(
                f"r={radius:.1f} actual mean "
                # f"(mean err {means.mean():.3f} m, std {stds.mean():.3f} m)"
            ),
        )
        ax.plot(
            ref_x,
            ref_y,
            ":",
            color=color,
            linewidth=2.2,
            label=f"r={radius:.1f} reference",
        )
        # summary_rows.append(
        #     (
        #         radius,
        #         rmses.mean(),
        #         means.mean(),
        #         stds.mean(),
        #     )
        # )

    ax.set_title("Combined Lemniscate Trajectory Tracking by Radius")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)

    # summary = "\n".join(
    #     f"r={radius:.1f}: mean RMSE={rmse:.3f} m, mean={mean:.3f} m, std={std:.3f} m"
    #     for radius, rmse, mean, std in summary_rows
    # )
    # ax.text(
    #     0.02,
    #     0.02,
    #     # summary,
    #     transform=ax.transAxes,
    #     fontsize=8,
    #     va="bottom",
    #     bbox={"facecolor": "white", "edgecolor": "0.8", "alpha": 0.85},
    # )

    output_dir = Path("Tuning_lemniscate")
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / "combined_lemniscate_mean_reference_by_radius.png"
    fig.tight_layout()
    fig.savefig(output_path, dpi=300)
    print(os.path.abspath(output_path))


if __name__ == "__main__":
    main()
