import argparse
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev


DEFAULT_QCAR2 = "hardware_results_test_qcar21/May11qcar2togethertrail3.npz"
DEFAULT_QCAR1 = "hardware_results_qcar1/May11qcar2togethertrail3.npz"
DEFAULT_OUTPUT = "May15_qcar1_qcar2_togethertrial3_combined.gif"
DEFAULT_STATIC_OUTPUT = "Tuning_lemniscate/May15_qcar1_qcar2_togethertrial3_trajectory_overlay.png"

VEHICLE_LENGTH = 0.425
VEHICLE_WIDTH = 0.192
WB = 0.256


def load_run(path, label, color):
    data = np.load(path, allow_pickle=True)
    hist = np.asarray(data["car1_history"], dtype=float)
    if hist.ndim != 2 or hist.shape[1] < 7:
        raise ValueError(f"{path} does not contain a usable car1_history array")

    abs_t = hist[:, 6]
    rel_t = abs_t - abs_t[0]
    return {
        "path": Path(path),
        "label": label,
        "color": color,
        "x": hist[:, 0],
        "y": hist[:, 1],
        "yaw": hist[:, 2],
        "v": hist[:, 3],
        "abs_t": abs_t,
        "rel_t": rel_t,
        "ref_x": np.asarray(data["reference_x"], dtype=float).reshape(-1),
        "ref_y": np.asarray(data["reference_y"], dtype=float).reshape(-1),
    }


def rotation_from_reference(run):
    pts = np.column_stack((run["ref_x"], run["ref_y"]))
    centered = pts - pts.mean(axis=0)
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    major_axis = vh[0]
    major_angle = math.atan2(major_axis[1], major_axis[0])
    vertical_angle = math.pi / 2.0

    # Rotate the largest reference axis onto the y-axis. Normalize to [-pi/2, pi/2]
    # so we apply the small correction instead of flipping the plot.
    angle = vertical_angle - major_angle
    while angle > math.pi / 2.0:
        angle -= math.pi
    while angle < -math.pi / 2.0:
        angle += math.pi
    return angle


def rotate_run(run, angle, origin):
    c, s = math.cos(angle), math.sin(angle)

    def rotate_xy(x, y):
        x0 = np.asarray(x) - origin[0]
        y0 = np.asarray(y) - origin[1]
        return c * x0 - s * y0, s * x0 + c * y0

    run["x"], run["y"] = rotate_xy(run["x"], run["y"])
    run["ref_x"], run["ref_y"] = rotate_xy(run["ref_x"], run["ref_y"])
    run["yaw"] = run["yaw"] + angle


def smooth_display_path(x, y, points=700, smoothing=0.001):
    """Return a spline-smoothed path for plotting only."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    keep = np.r_[True, np.hypot(np.diff(x), np.diff(y)) > 1e-8]
    x = x[keep]
    y = y[keep]
    if len(x) < 4:
        return x, y

    try:
        tck, _ = splprep([x, y], s=smoothing * len(x), k=min(3, len(x) - 1))
        u = np.linspace(0.0, 1.0, points)
        xs, ys = splev(u, tck)
        return np.asarray(xs), np.asarray(ys)
    except Exception:
        return x, y


def draw_path_band(ax, x, y, color, width=0.18, alpha=0.12, zorder=1):
    """Draw a soft constant-width visual band around a path."""
    ax.plot(
        x,
        y,
        color=color,
        linewidth=width * 72,
        alpha=alpha,
        solid_capstyle="round",
        solid_joinstyle="round",
        zorder=zorder,
    )


def state_and_trail_at_time(run, t_now, trail_dt=0.05):
    yaw_unwrapped = np.unwrap(run["yaw"])

    if t_now <= run["abs_t"][0]:
        return run["x"][0], run["y"][0], run["yaw"][0], run["x"][:1], run["y"][:1]
    if t_now >= run["abs_t"][-1]:
        t_trail = np.arange(run["abs_t"][0], run["abs_t"][-1], trail_dt)
        t_trail = np.r_[t_trail, run["abs_t"][-1]]
        trail_x = np.interp(t_trail, run["abs_t"], run["x"])
        trail_y = np.interp(t_trail, run["abs_t"], run["y"])
        return run["x"][-1], run["y"][-1], run["yaw"][-1], trail_x, trail_y

    x_now = float(np.interp(t_now, run["abs_t"], run["x"]))
    y_now = float(np.interp(t_now, run["abs_t"], run["y"]))
    yaw_now = float(np.interp(t_now, run["abs_t"], yaw_unwrapped))
    t_trail = np.arange(run["abs_t"][0], t_now, trail_dt)
    t_trail = np.r_[t_trail, t_now]
    trail_x = np.interp(t_trail, run["abs_t"], run["x"])
    trail_y = np.interp(t_trail, run["abs_t"], run["y"])
    return x_now, y_now, yaw_now, trail_x, trail_y


def draw_car(ax, x, y, yaw, color):
    dx = -0.05
    dy = -VEHICLE_WIDTH / 2.0
    corners_car = np.array(
        [
            [dx, dy],
            [dx + VEHICLE_LENGTH, dy],
            [dx + VEHICLE_LENGTH, dy + VEHICLE_WIDTH],
            [dx, dy + VEHICLE_WIDTH],
        ]
    )
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    corners_world = (rot @ corners_car.T).T + np.array([x, y])
    body = plt.Polygon(
        corners_world,
        closed=True,
        edgecolor=color,
        facecolor=color,
        alpha=0.35,
        zorder=6,
    )
    arrow = mpatches.FancyArrow(
        x,
        y,
        WB * c,
        WB * s,
        width=0.015,
        head_width=0.04,
        head_length=0.03,
        color=color,
        zorder=7,
    )
    ax.add_patch(body)
    ax.add_patch(arrow)
    return body, arrow


def plot_combined_trajectory(qcar1_path=DEFAULT_QCAR1, qcar2_path=DEFAULT_QCAR2,
                             output_path=DEFAULT_STATIC_OUTPUT, straighten=True):
    """Plot QCar1 and QCar2 trajectories on the same reference path."""
    run_qcar1 = load_run(qcar1_path, "QCar1", "tab:green")
    run_qcar2 = load_run(qcar2_path, "QCar2", "tab:blue")

    if straighten:
        angle = rotation_from_reference(run_qcar2)
        origin = np.array([run_qcar2["ref_x"].mean(), run_qcar2["ref_y"].mean()])
        rotate_run(run_qcar1, angle, origin)
        rotate_run(run_qcar2, angle, origin)

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    ref_x, ref_y = smooth_display_path(run_qcar1["ref_x"], run_qcar1["ref_y"], smoothing=0.0001)
    qcar1_x, qcar1_y = smooth_display_path(run_qcar1["x"], run_qcar1["y"], smoothing=0.0006)
    qcar2_x, qcar2_y = smooth_display_path(run_qcar2["x"], run_qcar2["y"], smoothing=0.0006)

    fig, ax = plt.subplots(figsize=(7, 8.5))
    ax.grid(True, color="0.82", linewidth=1.0, alpha=0.7)

    # Desired/reference path, matching the style of the tuning plots.
    draw_path_band(ax, qcar1_x, qcar1_y, "tab:green", width=0.16, alpha=0.10, zorder=1)
    draw_path_band(ax, qcar2_x, qcar2_y, "tab:blue", width=0.16, alpha=0.10, zorder=1)
    ax.plot(
        ref_x,
        ref_y,
        linestyle=":",
        color="#ff7f0e",
        linewidth=3.0,
        label="Reference",
        zorder=2,
    )

    # Combined actual paths.
    ax.plot(
        qcar1_x,
        qcar1_y,
        color="tab:green",
        linewidth=3.0,
        label="QCar1 actual path",
        zorder=3,
    )
    ax.plot(
        qcar2_x,
        qcar2_y,
        color="tab:blue",
        linewidth=3.0,
        label="QCar2 actual path",
        zorder=3,
    )

    ax.set_title("QCar1 + QCar2 Trajectory Overlay")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.78)

    fig.tight_layout()
    fig.savefig(output_path, dpi=170, bbox_inches="tight")
    plt.close(fig)
    return output_path


def build_animation(run_a, run_b, output, fps, title, trim_start):
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.4)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    ax.plot(
        run_a["ref_x"],
        run_a["ref_y"],
        "--",
        color="salmon",
        linewidth=1.2,
        alpha=0.7,
        label="Reference path",
        zorder=1,
    )

    all_x = np.concatenate([run_a["ref_x"], run_a["x"], run_b["x"]])
    all_y = np.concatenate([run_a["ref_y"], run_a["y"], run_b["y"]])
    margin = 0.7
    ax.set_xlim(float(np.min(all_x) - margin), float(np.max(all_x) + margin))
    ax.set_ylim(float(np.min(all_y) - margin), float(np.max(all_y) + margin))

    line_a, = ax.plot(
        [],
        [],
        "-",
        color=run_a["color"],
        linewidth=1.5,
        solid_capstyle="round",
        solid_joinstyle="round",
        label=f"{run_a['label']} trail",
        zorder=3,
    )
    line_b, = ax.plot(
        [],
        [],
        "-",
        color=run_b["color"],
        linewidth=1.5,
        solid_capstyle="round",
        solid_joinstyle="round",
        label=f"{run_b['label']} trail",
        zorder=3,
    )
    dot_a = ax.scatter([], [], s=40, color=run_a["color"], zorder=8, label=run_a["label"])
    dot_b = ax.scatter([], [], s=40, color=run_b["color"], zorder=8, label=run_b["label"])
    ax.legend(loc="upper right", fontsize=7, framealpha=0.6)

    car_artists = []
    log_start_t = float(min(run_a["abs_t"][0], run_b["abs_t"][0]))
    end_t = float(max(run_a["abs_t"][-1], run_b["abs_t"][-1]))
    start_t = min(log_start_t + trim_start, end_t)
    frame_times = np.arange(start_t, end_t + 1.0 / fps, 1.0 / fps)

    def update_run_artists(run, t_now, line, dot):
        x_now, y_now, yaw_now, trail_x, trail_y = state_and_trail_at_time(run, t_now)
        line.set_data(trail_x, trail_y)
        dot.set_offsets([[x_now, y_now]])
        car_artists.extend(draw_car(ax, x_now, y_now, yaw_now, run["color"]))

    def update(frame_i):
        while car_artists:
            artist = car_artists.pop()
            artist.remove()

        t_now = float(frame_times[frame_i])

        update_run_artists(run_a, t_now, line_a, dot_a)
        update_run_artists(run_b, t_now, line_b, dot_b)

        ax.set_title(title)
        return [line_a, line_b, dot_a, dot_b, *car_artists]

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(frame_times),
        interval=1000.0 / fps,
        blit=False,
    )
    writer = animation.PillowWriter(fps=fps)
    ani.save(output, writer=writer)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Replay two QCar NPZ logs together as one live-plot GIF."
    )
    parser.add_argument("--qcar2", default=DEFAULT_QCAR2, help="NPZ file for QCar2")
    parser.add_argument("--qcar1", default=DEFAULT_QCAR1, help="NPZ file for QCar1")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help="Output GIF path")
    parser.add_argument("--static-output", default=DEFAULT_STATIC_OUTPUT, help="Output PNG path for the static combined plot")
    parser.add_argument("--fps", type=int, default=8, help="GIF frames per second")
    parser.add_argument("--title", default="QCar1 + QCar2 Together Trial 3", help="Plot title")
    parser.add_argument(
        "--trim-start",
        type=float,
        default=20.0,
        help="Seconds to skip from the beginning of the combined timeline",
    )
    parser.add_argument(
        "--sync",
        choices=("absolute", "relative"),
        default="absolute",
        help="absolute uses saved timestamps; relative starts both logs at t=0",
    )
    parser.add_argument(
        "--no-straighten",
        action="store_true",
        help="Keep the original tilted coordinate frame",
    )
    parser.add_argument(
        "--static-only",
        action="store_true",
        help="Only save the static combined trajectory plot",
    )
    args = parser.parse_args()

    static_path = plot_combined_trajectory(
        qcar1_path=args.qcar1,
        qcar2_path=args.qcar2,
        output_path=args.static_output,
        straighten=not args.no_straighten,
    )
    print(f"Saved static combined plot to {static_path.resolve()}")
    if args.static_only:
        return

    run_qcar2 = load_run(args.qcar2, "QCar2", "tab:blue")
    run_qcar1 = load_run(args.qcar1, "QCar1", "tab:green")
    if not args.no_straighten:
        angle = rotation_from_reference(run_qcar2)
        origin = np.array([run_qcar2["ref_x"].mean(), run_qcar2["ref_y"].mean()])
        rotate_run(run_qcar2, angle, origin)
        rotate_run(run_qcar1, angle, origin)
        print(f"Straightened plot by rotating {math.degrees(angle):.2f} degrees")
    if args.sync == "relative":
        for run in (run_qcar2, run_qcar1):
            run["abs_t"] = run["rel_t"]
    build_animation(run_qcar2, run_qcar1, args.output, args.fps, args.title, args.trim_start)
    print(f"Saved combined GIF to {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
