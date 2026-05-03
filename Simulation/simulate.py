"""
simulate.py — Standalone MPC simulation for QCar (no ROS / no hardware)

Replicates publish.py's trajectory setup and control loop but uses
the bicycle-model update_state() instead of NatNet pose feedback.

Run from the QCar/ directory:
    python -m Simulation.simulate
"""
import math
import time
import os
import sys
import pathlib
import numpy as np

# Allow running as a plain script (python Simulation/simulate.py)
# as well as a module (python -m Simulation.simulate)
sys.path.insert(0, str(pathlib.Path(__file__).parent))

try:
    from .qcar_params import (
        MAX_SPEED, MIN_SPEED, MAX_STEER, DT, RADIUS, TARGET_SPEED,
        MAX_TIME, DS, LENGTH,
    )
    from .mpcspeed_steercontrol import (
        State, update_state,
        calc_ref_trajectory, iterative_linear_mpc_control,
        calc_speed_profile, smooth_yaw, GOAL_DIS,
    )
    from .trajectory import get_trajectory
    from .live_plotter import LivePlotter
except ImportError:
    from qcar_params import (
        MAX_SPEED, MIN_SPEED, MAX_STEER, DT, RADIUS, TARGET_SPEED,
        MAX_TIME, DS, LENGTH,
    )
    from mpcspeed_steercontrol import (
        State, update_state,
        calc_ref_trajectory, iterative_linear_mpc_control,
        calc_speed_profile, smooth_yaw, GOAL_DIS,
    )
    from trajectory import get_trajectory
    from live_plotter import LivePlotter
# ── Simulation configuration 
TRAJECTORY_TYPE = "lemniscate"   # "circle", "straight", or "lemniscate"
CLOCKWISE       = True           # only used for "circle"

# Car starting pose (world frame)
START_X   = 0.0   # [m]
START_Y   = 0.0   # [m]
START_YAW = 0.0   # [rad]  (0 = pointing in +x direction)

SAVE_GIF    = True
SAVE_RESULTS = True


def _build_trajectory(trajectory_type, clockwise, start_x, start_y, start_yaw,
                      radius, dl):
    direction_sign = -1 if clockwise else 1

    if trajectory_type == "circle":
        # Replicate publish.py center calculation
        if clockwise:
            center_x = start_x + radius * math.sin(start_yaw)
            center_y = start_y - radius * math.cos(start_yaw)
        else:
            center_x = start_x - radius * math.sin(start_yaw)
            center_y = start_y + radius * math.cos(start_yaw)

        cx, cy, cyaw, ck, _ = get_trajectory(
            "circle",
            radius=radius, ds=dl,
            center_x=center_x, center_y=center_y,
            direction_sign=direction_sign,
        )

        # Roll waypoints so index 0 is closest to start pose
        min_dist, start_idx = float("inf"), 0
        for i in range(len(cx)):
            d = math.hypot(cx[i] - start_x, cy[i] - start_y)
            if d < min_dist:
                min_dist, start_idx = d, i
        cx   = np.roll(cx,   -start_idx).tolist()
        cy   = np.roll(cy,   -start_idx).tolist()
        cyaw = np.roll(cyaw, -start_idx).tolist()
        ck   = np.roll(ck,   -start_idx).tolist()
        cyaw = smooth_yaw(cyaw)

        # Close the loop (publish.py adds a repeated first point)
        cx.append(cx[0])
        cy.append(cy[0])
        cyaw.append(cyaw[0] + direction_sign * 2.0 * math.pi)
        ck.append(ck[0])
        cyaw = smooth_yaw(cyaw)

    elif trajectory_type == "lemniscate":
        scale = radius  # reuse radius param as the figure-8 half-width
        # Place the crossing point so the car starts at the tangent-aligned
        # entry of the right lobe.  At that entry the lemniscate tangent is
        # exactly parallel to start_yaw, giving zero initial heading error.
        # Offset from lemniscate center to the entry point (unrotated frame):
        #   Δx = -√6/4·scale,  Δy = +√2/4·scale
        # so center = start - R(start_yaw)·(Δx, Δy):
        C6 = math.sqrt(6.0) / 4.0  # ≈ 0.612
        C2 = math.sqrt(2.0) / 4.0  # ≈ 0.354
        cos_y, sin_y = math.cos(start_yaw), math.sin(start_yaw)
        center_x = start_x + scale * ( C6 * cos_y + C2 * sin_y)
        center_y = start_y + scale * ( C6 * sin_y - C2 * cos_y)

        cx, cy, cyaw, ck, _ = get_trajectory(
            "lemniscate",
            scale=scale, ds=dl,
            center_x=center_x, center_y=center_y,
            start_angle=start_yaw,
        )

        # Roll so index 0 is the waypoint closest to the car's start
        min_dist, start_idx = float("inf"), 0
        for i in range(len(cx)):
            d = math.hypot(cx[i] - start_x, cy[i] - start_y)
            if d < min_dist:
                min_dist, start_idx = d, i
        cx   = np.roll(cx,   -start_idx).tolist()
        cy   = np.roll(cy,   -start_idx).tolist()
        cyaw = np.roll(cyaw, -start_idx).tolist()
        ck   = np.roll(ck,   -start_idx).tolist()
        cyaw = smooth_yaw(cyaw)

    else:  # straight
        center_x, center_y = start_x, start_y
        cx, cy, cyaw, ck, _ = get_trajectory(
            "straight",
            length=LENGTH, ds=dl,
            start_x=start_x, start_y=start_y,
            angle=start_yaw,
        )

    return cx, cy, cyaw, ck, center_x, center_y


def run_simulation(
    trajectory_type=TRAJECTORY_TYPE,
    clockwise=CLOCKWISE,
    start_x=START_X,
    start_y=START_Y,
    start_yaw=START_YAW,
    radius=RADIUS,
    dl=DS,
    target_speed=TARGET_SPEED,
    max_time=MAX_TIME,
    save_gif=SAVE_GIF,
    save_results=SAVE_RESULTS,
):
    print(f"[sim] trajectory={trajectory_type}, {'CW' if clockwise else 'CCW'}, "
          f"radius={radius}m, target_speed={target_speed}m/s")

    cx, cy, cyaw, ck, center_x, center_y = _build_trajectory(
        trajectory_type, clockwise, start_x, start_y, start_yaw, radius, dl
    )
    print(f"[sim] Path center/origin: ({center_x:.3f}, {center_y:.3f})")
    print(f"[sim] Waypoints: {len(cx)}")

    sp = calc_speed_profile(cx, cy, cyaw, target_speed=target_speed)

    # Arc-length array for progress-based termination (same as publish.py)
    path_s = [0.0]
    for i in range(1, len(cx)):
        path_s.append(path_s[-1] + math.hypot(cx[i] - cx[i - 1], cy[i] - cy[i - 1]))
    path_length = path_s[-1]
    print(f"[sim] Path length: {path_length:.3f}m")

    # Start state — initial speed is MIN_SPEED so MPC has a valid linearisation point
    state = State(x=start_x, y=start_y, yaw=start_yaw, v=MIN_SPEED)

    run_label = (f"QCar SIM — {trajectory_type.capitalize()} "
                 f"{'CW' if clockwise else 'CCW'} — {time.strftime('%H:%M:%S')}")
    plotter = LivePlotter(cx, cy, cyaw, title=run_label)

    target_ind = 0
    oa, odelta = None, None
    accel, steer = 0.0, 0.0   # safe defaults; held on MPC infeasibility
    t_sim      = 0.0
    history    = []   # [x, y, yaw, v, steer, accel, t_sim]

    while t_sim <= max_time:
        xref, target_ind, dref = calc_ref_trajectory(
            state, cx, cy, cyaw, ck, sp, dl, target_ind
        )

        # Progress-based termination (replicates publish.py)
        path_progress  = path_s[min(target_ind, len(path_s) - 1)]
        path_remaining = path_length - path_progress
        if target_ind >= len(cx) - 1 or path_remaining <= GOAL_DIS:
            print(f"[sim] path complete — t={t_sim:.2f}s, "
                  f"progress={path_progress:.3f}/{path_length:.3f}m")
            break

        x_ref = float(xref[0, 0])
        y_ref = float(xref[1, 0])
        yaw_ref = float(xref[3, 0])
        v_ref   = float(xref[2, 0])
        print(f"t={t_sim:.2f}s | x={state.x:.3f} xref={x_ref:.3f} "
              f"y={state.y:.3f} yref={y_ref:.3f} "
              f"yaw={state.yaw:.3f} yawref={yaw_ref:.3f} "
              f"v={state.v:.3f} vref={v_ref:.3f}")

        x0 = [state.x, state.y, state.v, state.yaw]
        oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(
            xref, x0, dref, oa, odelta
        )

        if oa is None or odelta is None:
            print(f"[sim] MPC infeasible at t={t_sim:.2f}s - holding last command")
            ox, oy = None, None
            # accel, steer keep their values from the previous iteration
        else:
            accel = float(oa[0])
            steer = float(odelta[0])

        # Advance simulated state with bicycle model
        state = update_state(state, accel, steer)
        t_sim += DT

        history.append([state.x, state.y, state.yaw, state.v, steer, accel, t_sim])

        plotter.update(
            state_x      = state.x,
            state_y      = state.y,
            state_yaw    = state.yaw,
            state_v      = state.v,
            ox           = ox,
            oy           = oy,
            xref         = xref,
            target_ind   = target_ind,
            elapsed_time = t_sim,
        )

    else:
        print(f"[sim] max_time ({max_time}s) reached")

    plotter.close()

    if history:
        hist_arr = np.array(history)         # shape (N, 7)
        cx_arr   = np.array(cx)
        cy_arr   = np.array(cy)
        # cross-track error: distance from each recorded position to the nearest waypoint
        cte = np.array([
            np.min(np.hypot(cx_arr - hx, cy_arr - hy))
            for hx, hy in zip(hist_arr[:, 0], hist_arr[:, 1])
        ])
        print(f"[sim] Cross-track error (m) | "
              f"mean={np.mean(cte):.4f}  "
              f"std={np.std(cte):.4f}  "
              f"max={np.max(cte):.4f}  "
              f"RMS={np.sqrt(np.mean(cte**2)):.4f}")

    tag = f"sim_{trajectory_type}_{'cw' if clockwise else 'ccw'}_{time.strftime('%Y%m%d_%H%M%S')}"

    if save_gif:
        gif_path = os.path.join(os.path.dirname(__file__), "..", f"{tag}.gif")
        plotter.save(gif_path)

    if save_results and history:
        hist_arr = np.array(history)
        npz_path = os.path.join(os.path.dirname(__file__), "..", f"{tag}.npz")
        np.savez_compressed(
            npz_path,
            car1_history     = hist_arr,
            reference_x      = np.array(cx),
            reference_y      = np.array(cy),
            reference_yaw    = np.array(cyaw),
            reference_curvature = np.array(ck),
            center_x         = center_x,
            center_y         = center_y,
            radius           = radius,
            target_speed     = target_speed,
            ds               = dl,
            path_length      = path_length,
            circle_direction = "cw" if clockwise else "ccw",
            max_time         = max_time,
        )
        print(f"[sim] results saved → {npz_path}")

    return history


if __name__ == "__main__":
    run_simulation()
