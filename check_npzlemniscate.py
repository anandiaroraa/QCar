import glob
import os
import sys

import matplotlib.pyplot as plt
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from MPC_Hardware2.qcar_params import (  # noqa: E402
    MAX_DSTEER,
    MAX_SPEED,
    MAX_STEER,
    MIN_SPEED,
    RADIUS,
    TARGET_SPEED,
    WB,
)


npz_pattern = os.path.join(script_dir, "hardware_results_qcar1", "*.npz")
files = sorted(glob.glob(npz_pattern), key=os.path.getmtime)
if not files:
    raise FileNotFoundError(f"No .npz files found for pattern: {npz_pattern}")

latest = files[-1]
print("Loading:", latest)

d = np.load(latest, allow_pickle=True)
print("\nKeys in npz file:", list(d.keys()))

hist = d["car1_history"]
print(f"car1_history shape: {hist.shape}")
if hist.shape[1] >= 8:
    print("Columns: [x, y, yaw, v_mpc_clipped, steer, speed_cmd, timestamp, v_raw_optitrack]")
else:
    print("Columns: [x, y, yaw, v_mpc_clipped, steer, speed_cmd, timestamp]")
    print("NOTE: old log file has no raw OptiTrack speed column; using clipped MPC speed as fallback.")

x = hist[:, 0]
y = hist[:, 1]
yaw = hist[:, 2]
v_mpc = hist[:, 3]
steer = hist[:, 4]
speed_cmd = hist[:, 5]
timestamp = hist[:, 6]
v_raw = hist[:, 7] if hist.shape[1] >= 8 else v_mpc

if "reference_x" not in d or "reference_y" not in d:
    raise KeyError("This checker needs saved reference_x/reference_y arrays for the lemniscate.")

wp_x = np.asarray(d["reference_x"]).reshape(-1)
wp_y = np.asarray(d["reference_y"]).reshape(-1)
wp_yaw = np.asarray(d["reference_yaw"]).reshape(-1) if "reference_yaw" in d else None

radius = float(d["radius"]) if "radius" in d else RADIUS
target_speed = float(d["target_speed"]) if "target_speed" in d else TARGET_SPEED
center_x = float(d["center_x"]) if "center_x" in d else float(wp_x[0])
center_y = float(d["center_y"]) if "center_y" in d else float(wp_y[0])

# Tracking error: distance from each measured pose to the nearest saved reference waypoint.
dists = np.sqrt((x[:, np.newaxis] - wp_x[np.newaxis, :]) ** 2 + (y[:, np.newaxis] - wp_y[np.newaxis, :]) ** 2)
nearest_idx = np.argmin(dists, axis=1)
tracking_error = dists[np.arange(len(x)), nearest_idx]

dt_samples = np.diff(timestamp)
t_rel = timestamp - timestamp[0]

yaw_unwrapped = np.unwrap(yaw)
yaw_rate_actual = np.divide(
    np.diff(yaw_unwrapped),
    dt_samples,
    out=np.zeros_like(dt_samples),
    where=dt_samples > 0,
)
yaw_rate_pred = (v_raw[:-1] / WB) * np.tan(steer[:-1])

rmse = float(np.sqrt(np.mean(tracking_error**2)))
mean_error = float(tracking_error.mean())
std_error = float(tracking_error.std())
max_error = float(tracking_error.max())
min_error = float(tracking_error.min())
total_time = float(timestamp[-1] - timestamp[0])
avg_speed_cmd = float(np.mean(speed_cmd))
avg_speed_raw = float(np.mean(np.abs(v_raw)))
avg_speed_mpc = float(np.mean(np.abs(v_mpc)))

yaw_rate_err = yaw_rate_actual - yaw_rate_pred
yaw_rate_rmse_deg = float(np.degrees(np.sqrt(np.mean(yaw_rate_err**2))))
yaw_rate_mae_deg = float(np.degrees(np.mean(np.abs(yaw_rate_err))))
if np.std(yaw_rate_actual) > 1e-9 and np.std(yaw_rate_pred) > 1e-9:
    yaw_rate_corr = float(np.corrcoef(yaw_rate_actual, yaw_rate_pred)[0, 1])
else:
    yaw_rate_corr = float("nan")

fig, axs = plt.subplots(1, 4, figsize=(24, 5))

axs[0].plot(wp_x, wp_y, "r--", linewidth=1.5, label="Reference")
axs[0].plot(wp_x, wp_y, "y*", markersize=5, label="Reference waypoints")
axs[0].plot(x, y, "b-", label="Actual")
axs[0].plot(x[0], y[0], "go", markersize=10, label="Start")
axs[0].plot(x[-1], y[-1], "rs", markersize=10, label="End")
axs[0].plot(center_x, center_y, "kx", markersize=9, label="Center/crossing")
axs[0].set_title(
    "Lemniscate Tracking\n"
    f"RMSE={rmse:.3f}m  Mean={mean_error:.3f}m  Std={std_error:.3f}m  Max={max_error:.3f}m"
)
axs[0].legend()
axs[0].axis("equal")
axs[0].grid(True)

axs[1].plot(t_rel, v_raw, "b-", label="Raw measured speed")
axs[1].plot(t_rel, v_mpc, color="0.35", linestyle="--", label="MPC speed used")
axs[1].plot(t_rel, speed_cmd, "r-", label="Commanded speed")
axs[1].axhline(y=MAX_SPEED, color="g", linestyle="-", label="MAX_SPEED")
axs[1].axhline(y=MIN_SPEED, color="orange", linestyle="-", label="MIN_SPEED")
axs[1].axhline(y=target_speed, color="purple", linestyle="-", label="Target speed")
axs[1].set_xlabel("Time (s)")
axs[1].set_ylabel("Speed (m/s)")
axs[1].set_title("Speed Profile")
axs[1].legend()
axs[1].grid(True)

steer_deg = np.degrees(steer)
max_steer_deg = np.degrees(MAX_STEER)
axs[2].plot(t_rel, steer_deg, "m-", label="Steer angle")
axs[2].axhline(y=max_steer_deg, color="k", linestyle="--", label="+MAX_STEER")
axs[2].axhline(y=-max_steer_deg, color="k", linestyle="--", label="-MAX_STEER")
axs[2].set_xlabel("Time (s)")
axs[2].set_ylabel("Steering Angle (deg)")
axs[2].set_title("Steering Profile")
axs[2].legend()
axs[2].grid(True)

dsteer = np.divide(np.diff(steer), dt_samples, out=np.zeros_like(dt_samples), where=dt_samples > 0)
dsteer_deg = np.degrees(dsteer)
max_dsteer_deg = np.degrees(MAX_DSTEER)
axs[3].plot(t_rel[1:], np.degrees(yaw_rate_actual), "c-", label="Yaw rate (measured)")
axs[3].plot(t_rel[1:], np.degrees(yaw_rate_pred), "m--", label="Yaw rate (pred from steer)")
axs[3].plot(t_rel[1:], dsteer_deg, color="0.6", linestyle=":", label="Steer rate")
axs[3].axhline(y=max_dsteer_deg, color="k", linestyle="--", label="+MAX_DSTEER")
axs[3].axhline(y=-max_dsteer_deg, color="k", linestyle="--", label="-MAX_DSTEER")
axs[3].set_xlabel("Time (s)")
axs[3].set_ylabel("Rate (deg/s)")
axs[3].set_title("Yaw-Rate Consistency Check")
axs[3].legend()
axs[3].grid(True)

plt.subplots_adjust(left=0.22)
plt.tight_layout()

tuning_dir = os.path.join(script_dir, "Tuning_lemniscate")
os.makedirs(tuning_dir, exist_ok=True)
base_name = os.path.splitext(os.path.basename(latest))[0]
save_path = os.path.join(tuning_dir, f"{base_name}_plot.png")
plt.savefig(save_path)
plt.show()

print(f"Trajectory type: {d['trajectory_type'] if 'trajectory_type' in d else 'unknown'}")
print(f"Reference waypoints: {len(wp_x)}")
print(f"Nearest-reference index range: {nearest_idx.min()} to {nearest_idx.max()}")
print(f"RMSE: {rmse:.4f} m")
print(f"Mean: {mean_error:.4f} m")
print(f"Std:  {std_error:.4f} m")
print(f"Max:  {max_error:.4f} m")
print(f"Min:  {min_error:.4f} m")
print(f"Duration: {total_time:.2f} s")
print(f"Avg commanded speed: {avg_speed_cmd:.4f} m/s")
print(f"Avg raw measured speed: {avg_speed_raw:.4f} m/s")
print(f"Avg MPC speed used: {avg_speed_mpc:.4f} m/s")
print(f"Yaw-rate RMSE (actual vs pred from steer): {yaw_rate_rmse_deg:.2f} deg/s")
print(f"Yaw-rate MAE  (actual vs pred from steer): {yaw_rate_mae_deg:.2f} deg/s")
print(f"Yaw-rate correlation: {yaw_rate_corr:.3f}")
print(f"Saved to {save_path}")
