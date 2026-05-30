"""
live_plotter.py  –  Real-time MPC path tracking visualizer for QCar hardware

Drop-in for the simulation's show_animation block.
Runs matplotlib in a background thread so it never blocks the ROS control loop.

Usage in shared2.py / publish.py
─────────────────────────────────
    from live_plotter import LivePlotter

    plotter = LivePlotter(cx, cy, cyaw, title="QCar MPC – Hardware Run")

    # inside your control loop, after you get a new pose + MPC solution:
    plotter.update(
        state_x   = state.x,
        state_y   = state.y,
        state_yaw = state.yaw,
        state_v   = state.v,
        ox=ox, oy=oy,                   # MPC predicted horizon (can be None)
        xref=xref,                      # (NX, T+1) reference trajectory array
        target_ind=target_ind,          # current target waypoint index
        elapsed_time=elapsed_time,      # float seconds since run start
    )

    # at the end of the run:
    plotter.close()
    plotter.save("run_plot.gif")        # optional – saves the final animation
"""

import math
import time
import numpy as np
import matplotlib
try:
    matplotlib.use("TkAgg")
except Exception:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from PIL import Image


# ──────────────────────────────────────────────────────────────────────────────
# QCar body dimensions  (metres, matching your mpcspeed_steercontrol.py)
# ──────────────────────────────────────────────────────────────────────────────
_VEHICLE_LENGTH  = 0.425
_VEHICLE_WIDTH   = 0.192
_WB              = 0.256   # wheel-base


def _draw_car_patch(ax, x, y, yaw, color="tab:blue"):
    """Return a rectangle patch representing the QCar body at (x,y,yaw)."""
    # rear-axle is the reference point; place rect so centre is at rear-axle
    w, l = _VEHICLE_WIDTH, _VEHICLE_LENGTH
    # bottom-left corner in car frame
    dx = -0.05          # small rear overhang behind rear axle
    dy = -w / 2.0

    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    # four corners in world frame
    corners_car = np.array([
        [dx,       dy],
        [dx + l,   dy],
        [dx + l,   dy + w],
        [dx,       dy + w],
    ])
    R = np.array([[cos_y, -sin_y],
                  [sin_y,  cos_y]])
    corners_world = (R @ corners_car.T).T + np.array([x, y])

    patch = plt.Polygon(corners_world, closed=True,
                        edgecolor=color, facecolor=color, alpha=0.35, zorder=5)
    # heading arrow
    arrow = mpatches.FancyArrow(
        x, y,
        _WB * cos_y, _WB * sin_y,
        width=0.015, head_width=0.04, head_length=0.03,
        color=color, zorder=6
    )
    return patch, arrow


# ──────────────────────────────────────────────────────────────────────────────
class LivePlotter:
    """
    Thread-safe live plotter for the QCar MPC hardware loop.

    Parameters
    ----------
    cx, cy, cyaw : list/array
        Full reference path (world frame, metres).
    title : str
        Window / figure title.
    update_hz : float
        Maximum display refresh rate (default 10 Hz).
        The control loop can call update() faster; frames are dropped gracefully.
    """

    def __init__(self, cx, cy, cyaw, title="QCar MPC – Live", update_hz=10.0,
                 secondary_reference=None):
        self.cx   = list(cx)
        self.cy   = list(cy)
        self.cyaw = list(cyaw)
        self._has_secondary = secondary_reference is not None
        if self._has_secondary:
            self.cx2, self.cy2, self.cyaw2 = [
                list(values) for values in secondary_reference
            ]
        else:
            self.cx2, self.cy2, self.cyaw2 = [], [], []
        self._title = title
        self._min_dt = 1.0 / update_hz
        self._gif_frame_duration_ms = max(1, int(1000 / update_hz))

        # ── latest state snapshot ──
        self._latest = None
        self._last_draw_time = 0.0

        # trajectory history
        self._hist_x   = []
        self._hist_y   = []
        self._hist_x2  = []
        self._hist_y2  = []
        self._gif_frames = []
        self._gif_frame_times = []

        # ── figure setup on the main thread ──
        plt.ion()
        self._fig, self._ax = plt.subplots(figsize=(7, 7))
        try:
            self._fig.canvas.manager.set_window_title(self._title)
        except Exception:
            pass
        self._ax.set_aspect("equal")
        self._ax.grid(True, alpha=0.4)
        self._ax.set_xlabel("x [m]")
        self._ax.set_ylabel("y [m]")

        # static reference path
        self._ax.plot(self.cx, self.cy, "--", color="salmon",
                      linewidth=1.2, alpha=0.7,
                      label="QCar1 reference" if self._has_secondary else "Reference path",
                      zorder=1)
        if self._has_secondary:
            self._ax.plot(self.cx2, self.cy2, "--", color="mediumseagreen",
                          linewidth=1.2, alpha=0.8, label="QCar2 reference",
                          zorder=1)

        # reference yaw arrows (every Nth point so it's not cluttered)
        step = max(1, len(self.cx) // 30)
        for xi, yi, yawi in zip(self.cx[::step], self.cy[::step], self.cyaw[::step]):
            self._ax.annotate(
                "",
                xy=(xi + 0.12 * math.cos(yawi), yi + 0.12 * math.sin(yawi)),
                xytext=(xi, yi),
                arrowprops=dict(arrowstyle="->", color="salmon", lw=0.8),
                zorder=2,
            )
        if self._has_secondary:
            step2 = max(1, len(self.cx2) // 30)
            for xi, yi, yawi in zip(self.cx2[::step2], self.cy2[::step2], self.cyaw2[::step2]):
                self._ax.annotate(
                    "",
                    xy=(xi + 0.12 * math.cos(yawi), yi + 0.12 * math.sin(yawi)),
                    xytext=(xi, yi),
                    arrowprops=dict(arrowstyle="->", color="mediumseagreen", lw=0.8),
                    zorder=2,
                )

        # dynamic artists (updated every frame)
        self._line_traj, = self._ax.plot([], [], "-", color="tab:blue",
                                         linewidth=1.5,
                                         label="QCar1 actual" if self._has_secondary else "Actual path",
                                         zorder=3)
        self._line_mpc,  = self._ax.plot([], [], "x", color="tab:red",
                                         markersize=4, alpha=0.8,
                                         label="QCar1 horizon" if self._has_secondary else "MPC horizon",
                                         zorder=4)
        self._line_xref, = self._ax.plot(
            [], [], "-o",
            color="deepskyblue",
            linewidth=2.0,
            markersize=7,
            markerfacecolor="gold",
            markeredgecolor="black",
            markeredgewidth=0.8,
            alpha=0.95,
            label="QCar1 lookahead" if self._has_secondary else "xref lookahead",
            zorder=5,
        )
        self._scat_pos = self._ax.scatter([], [], s=40, c="tab:blue",
                                          marker="o", zorder=6)
        if self._has_secondary:
            self._line_traj2, = self._ax.plot([], [], "-", color="tab:purple",
                                              linewidth=1.5, label="QCar2 actual",
                                              zorder=3)
            self._line_mpc2, = self._ax.plot([], [], "x", color="tab:orange",
                                             markersize=4, alpha=0.8,
                                             label="QCar2 horizon", zorder=4)
            self._line_xref2, = self._ax.plot(
                [], [], "-o", color="limegreen", linewidth=2.0, markersize=6,
                markerfacecolor="white", markeredgecolor="black",
                markeredgewidth=0.8, alpha=0.95, label="QCar2 lookahead",
                zorder=5,
            )
            self._scat_pos2 = self._ax.scatter([], [], s=40, c="tab:purple",
                                               marker="o", zorder=6)

        self._car_patch = None
        self._car_arrow = None
        self._car_patch2 = None
        self._car_arrow2 = None
        self._title_obj = self._ax.set_title(self._title)
        self._ax.legend(loc="upper right", fontsize=7, framealpha=0.6)
        plt.tight_layout()
        plt.show(block=False)

    # ── public API ────────────────────────────────────────────────────────────

    def update(self, state_x, state_y, state_yaw, state_v,
               ox=None, oy=None,
               xref=None,
               target_ind=None,
               elapsed_time=0.0,
               force=False,
               secondary_state=None, secondary_ox=None, secondary_oy=None,
               secondary_xref=None):
        """
        Call this once per MPC iteration (inside your control loop).
        Updates the live window on the main thread.
        """
        # accumulate history in the calling thread
        self._hist_x.append(state_x)
        self._hist_y.append(state_y)
        if self._has_secondary and secondary_state is not None:
            self._hist_x2.append(secondary_state[0])
            self._hist_y2.append(secondary_state[1])

        payload = dict(
            x=state_x, y=state_y, yaw=state_yaw, v=state_v,
            hist_x=list(self._hist_x),
            hist_y=list(self._hist_y),
            ox=list(ox) if ox is not None else None,
            oy=list(oy) if oy is not None else None,
            xref=xref.copy() if xref is not None else None,
            target_ind=target_ind,
            elapsed_time=elapsed_time,
            secondary_state=secondary_state,
            hist_x2=list(self._hist_x2),
            hist_y2=list(self._hist_y2),
            ox2=list(secondary_ox) if secondary_ox is not None else None,
            oy2=list(secondary_oy) if secondary_oy is not None else None,
            xref2=secondary_xref.copy() if secondary_xref is not None else None,
        )

        self._latest = payload

        now = time.time()
        if force or now - self._last_draw_time >= self._min_dt:
            self._redraw(payload)
            self._last_draw_time = now
            try:
                self._fig.canvas.flush_events()
            except Exception:
                pass

    def close(self):
        """Close the plot window."""
        try:
            plt.close(self._fig)
        except Exception:
            pass

    def save(self, filepath="live_plot_final.gif", extra_seconds=3.0):
        """Save the current figure to disk (call after close()).

        extra_seconds adds a frozen tail at the end of the GIF.
        """
        if filepath.lower().endswith(".gif"):
            if not self._gif_frames:
                print("[LivePlotter] no frames captured, nothing to save")
                return
            try:
                frames = list(self._gif_frames)
                if len(self._gif_frame_times) == len(frames) and len(frames) > 1:
                    durations = []
                    for t0, t1 in zip(self._gif_frame_times[:-1], self._gif_frame_times[1:]):
                        dt_ms = int(round((t1 - t0) * 1000.0))
                        durations.append(max(20, dt_ms))
                    durations.append(self._gif_frame_duration_ms)
                else:
                    durations = [self._gif_frame_durations_ms] * len(frames)
                if extra_seconds and extra_seconds > 0:
                    durations[-1] += int(round(extra_seconds * 1000.0))
                self._gif_frames[0].save(
                    filepath,
                    save_all=True,
                    append_images=frames[1:],
                    duration=durations,
                    optimize=False,
                )
                print(f"[LivePlotter] GIF saved → {filepath}  ({len(frames)} frames, +{extra_seconds:.1f}s tail)")
            except Exception as e:
                print(f"[LivePlotter] GIF save failed: {e}")
        else:
            try:
                self._fig.savefig(filepath, dpi=150, bbox_inches="tight")
                print(f"[LivePlotter] saved → {filepath}")
            except Exception as e:
                print(f"[LivePlotter] save failed: {e}")

    def _redraw(self, d):
        ax = self._ax

        # ── actual trajectory ──
        self._line_traj.set_data(d["hist_x"], d["hist_y"])

        # ── MPC predicted horizon ──
        if d["ox"] is not None and d["oy"] is not None:
            self._line_mpc.set_data(d["ox"], d["oy"])
        else:
            self._line_mpc.set_data([], [])

        # ── xref lookahead window (all T+1 reference points) ──
        if d["xref"] is not None:
            self._line_xref.set_data(d["xref"][0, :], d["xref"][1, :])
        else:
            self._line_xref.set_data([], [])

        # ── current position dot ──
        self._scat_pos.set_offsets(np.c_[d["x"], d["y"]])
        if self._has_secondary:
            self._line_traj2.set_data(d["hist_x2"], d["hist_y2"])
            if d["ox2"] is not None and d["oy2"] is not None:
                self._line_mpc2.set_data(d["ox2"], d["oy2"])
            else:
                self._line_mpc2.set_data([], [])
            if d["xref2"] is not None:
                self._line_xref2.set_data(d["xref2"][0, :], d["xref2"][1, :])
            else:
                self._line_xref2.set_data([], [])
            if d["secondary_state"] is not None:
                self._scat_pos2.set_offsets(np.c_[d["secondary_state"][0], d["secondary_state"][1]])

        # ── car body ──
        if self._car_patch is not None:
            self._car_patch.remove()
        if self._car_arrow is not None:
            self._car_arrow.remove()
        patch, arrow = _draw_car_patch(ax, d["x"], d["y"], d["yaw"])
        self._car_patch = ax.add_patch(patch)
        self._car_arrow = ax.add_patch(arrow)
        if self._has_secondary and d["secondary_state"] is not None:
            if self._car_patch2 is not None:
                self._car_patch2.remove()
            if self._car_arrow2 is not None:
                self._car_arrow2.remove()
            x2, y2, yaw2, _ = d["secondary_state"]
            patch2, arrow2 = _draw_car_patch(ax, x2, y2, yaw2, color="tab:purple")
            self._car_patch2 = ax.add_patch(patch2)
            self._car_arrow2 = ax.add_patch(arrow2)

        # ── auto-zoom: keep view centred on car with some margin ──
        margin = 1.5   # metres
        all_x = [d["x"]] + (d["ox"] or []) + d["hist_x"][-50:]
        all_y = [d["y"]] + (d["oy"] or []) + d["hist_y"][-50:]
        if self._has_secondary and d["secondary_state"] is not None:
            all_x += [d["secondary_state"][0]] + (d["ox2"] or []) + d["hist_x2"][-50:]
            all_y += [d["secondary_state"][1]] + (d["oy2"] or []) + d["hist_y2"][-50:]
        xmin, xmax = min(all_x) - margin, max(all_x) + margin
        ymin, ymax = min(all_y) - margin, max(all_y) + margin

        # also keep reference path visible
        xmin = min(xmin, min(self.cx) - margin)
        xmax = max(xmax, max(self.cx) + margin)
        ymin = min(ymin, min(self.cy) - margin)
        ymax = max(ymax, max(self.cy) + margin)
        if self._has_secondary:
            xmin = min(xmin, min(self.cx2) - margin)
            xmax = max(xmax, max(self.cx2) + margin)
            ymin = min(ymin, min(self.cy2) - margin)
            ymax = max(ymax, max(self.cy2) + margin)
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymin, ymax)

        # ── title ──
        speed_kmh = d["v"] * 3.6
        speed_label = f"v1={speed_kmh:.2f} km/h" if self._has_secondary else f"v={speed_kmh:.2f} km/h"
        if self._has_secondary and d["secondary_state"] is not None:
            speed_label += f"   v2={d['secondary_state'][3] * 3.6:.2f} km/h"
        self._title_obj.set_text(
            f"{self._title}   t={d['elapsed_time']:.1f}s   {speed_label}"
        )

        try:
            self._fig.canvas.draw()
            self._fig.canvas.flush_events()
            frame = np.asarray(self._fig.canvas.buffer_rgba(), dtype=np.uint8)[..., :3].copy()
            self._gif_frames.append(Image.fromarray(frame))
            self._gif_frame_times.append(float(d["elapsed_time"]))
        except Exception:
            pass
