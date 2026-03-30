import numpy as np
import matplotlib.pyplot as plt
import math
import glob
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from MPC_Hardware.qcar_params import (
    MAX_SPEED,
    MIN_SPEED,
    MAX_STEER,
    MAX_DSTEER,
    TARGET_SPEED,
    WB,
)

npz_pattern = os.path.join(script_dir, 'hardware_results_test1', '*.npz')
files = sorted(glob.glob(npz_pattern), key=os.path.getmtime)
if not files:
    raise FileNotFoundError(f"No .npz files found for pattern: {npz_pattern}")
latest = files[-1]
print("Loading:", latest)

d = np.load(latest, allow_pickle=True)
print("\nKeys in npz file:", list(d.keys()))

hist = d['car1_history']
print(f"car1_history shape: {hist.shape}")
print("Columns: [x, y, yaw, v, steer, speed, timestamp]")

x = hist[:, 0]
y = hist[:, 1]
yaw = hist[:, 2]
v = hist[:, 3]
steer = hist[:, 4]
speed_cmd = hist[:, 5]
timestamp = hist[:, 6]

theta_start = hist[0, 2]
target_speed = float(d['target_speed']) if 'target_speed' in d else TARGET_SPEED

# ── Reference path ──────────────────────────────────────────────────────────
# Step 1: load whatever was saved in the npz (may be short if LENGTH was small)
if 'reference_x' in d and 'reference_y' in d:
    saved_wp_x = np.asarray(d['reference_x']).reshape(-1)
    saved_wp_y = np.asarray(d['reference_y']).reshape(-1)
    have_saved_ref = True
else:
    have_saved_ref = False

# Step 2: compute how far the car actually traveled
actual_span = float(np.sqrt((x[-1] - x[0]) ** 2 + (y[-1] - y[0]) ** 2))

# Step 3: build intended straight-line reference direction.
# Use start yaw as the commanded direction (do not fit from actual trajectory).
fit_dir_x = math.cos(theta_start)
fit_dir_y = math.sin(theta_start)

# Build reference that starts at car start and covers full trajectory + 10% margin
s_vals = np.linspace(0.0, actual_span * 1.1, 500)
wp_x = x[0] + s_vals * fit_dir_x
wp_y = y[0] + s_vals * fit_dir_y

# Decide label based on whether the saved reference was long enough
if have_saved_ref:
    saved_span = float(np.sqrt((saved_wp_x[-1] - saved_wp_x[0]) ** 2 +
                               (saved_wp_y[-1] - saved_wp_y[0]) ** 2))
    if saved_span >= actual_span * 0.5:
        # Saved reference is long enough – use it directly
        wp_x = saved_wp_x
        wp_y = saved_wp_y
        path_label = 'Reference path (saved)'
    else:
        path_label = f'Reference (fitted – saved only {saved_span:.2f}m, car went {actual_span:.2f}m)'
else:
    path_label = 'Reference (fitted from actual trajectory)'

ref_x = wp_x
ref_y = wp_y

# ── Tracking errors ──────────────────────────────────────────────────────────
# 1) Nearest-waypoint distance to reference path (used for RMSE/Mean/Max/Min)
# 2) Signed lateral error to straight reference direction (diagnostic plot)
dx_all = x - x[0]
dy_all = y - y[0]
# signed lateral error (positive = left of direction of travel)
lateral_error_signed = dx_all * fit_dir_y - dy_all * fit_dir_x
lateral_error = np.abs(lateral_error_signed)

dists = np.sqrt((x[:, np.newaxis] - wp_x[np.newaxis, :]) ** 2 +
                (y[:, np.newaxis] - wp_y[np.newaxis, :]) ** 2)
tracking_error = np.min(dists, axis=1)
nearest_ref_idx = np.argmin(dists, axis=1)
xref_t = wp_x[nearest_ref_idx]
yref_t = wp_y[nearest_ref_idx]

# ── Yaw reference ────────────────────────────────────────────────────────────
if have_saved_ref and saved_span >= actual_span * 0.8 and 'reference_yaw' in d:
    wp_yaw = np.asarray(d['reference_yaw']).reshape(-1)
    if len(wp_yaw) != len(wp_x):
        wp_yaw = np.pad(wp_yaw, (0, max(0, len(wp_x) - len(wp_yaw))), mode='edge')[:len(wp_x)]
else:
    # Yaw is constant along a straight line
    ref_yaw_val = math.atan2(fit_dir_y, fit_dir_x)
    wp_yaw = np.full(len(wp_x), ref_yaw_val)

yawref_t = wp_yaw[nearest_ref_idx]

# ── Actual waypoints (downsampled for plotting) ───────────────────────────────
n_ref_wp = len(wp_x)
actual_wp_step = max(1, len(x) // n_ref_wp)
actual_wp_x = x[::actual_wp_step]
actual_wp_y = y[::actual_wp_step]

# ── Timing / steer-rate diagnostics ─────────────────────────────────────────
dt_samples = np.diff(timestamp)
t_rel = timestamp - timestamp[0]

yaw_unwrapped = np.unwrap(yaw)
yaw_rate_actual = np.divide(
    np.diff(yaw_unwrapped),
    dt_samples,
    out=np.zeros_like(dt_samples),
    where=dt_samples > 0,
)
yaw_rate_pred = (v[:-1] / WB) * np.tan(steer[:-1])

# ── Plot 1: path overview ────────────────────────────────────────────────────
rmse = float(np.sqrt(np.mean(tracking_error ** 2)))
mean_error = float(tracking_error.mean())
max_error = float(tracking_error.max())
min_error = float(tracking_error.min())

fig, ax = plt.subplots(1, 1, figsize=(6, 6))
ax.plot(ref_y, ref_x, 'r--', linewidth=1.5, label=path_label)
ax.plot(wp_y, wp_x, 'y*', markersize=4, label='Reference waypoints')
ax.plot(y, x, 'b-', linewidth=1.5, label='Actual')
ax.plot(actual_wp_y, actual_wp_x, 'c*', markersize=4, label='Actual waypoints')
ax.plot(y[0], x[0], 'go', markersize=10, label='Start')
ax.plot(y[-1], x[-1], 'rs', markersize=10, label='End')
ax.set_title(
    f'Straight Path Tracking\n'
    f'RMSE={rmse:.3f}m  Mean={mean_error:.3f}m  Max={max_error:.3f}m'
)
ax.set_xlabel('y [m]')
ax.set_ylabel('x [m]')
ax.legend(fontsize=8)
ax.axis('equal')
ax.grid(True)
plt.tight_layout()

output_dir = os.path.join(script_dir, 'Tuning_straight')
os.makedirs(output_dir, exist_ok=True)
base_name = os.path.splitext(os.path.basename(latest))[0]
save_path = os.path.join(output_dir, f'{base_name}_plot.png')
plt.savefig(save_path, dpi=150)
plt.show()

# ── Plot 2: lateral error over time ─────────────────────────────────────────
fig_lat, ax_lat = plt.subplots(figsize=(10, 3))
ax_lat.plot(t_rel, lateral_error_signed, 'b-', linewidth=1.2, label='Lateral error (signed)')
ax_lat.axhline(0, color='r', linestyle='--', linewidth=1)
ax_lat.fill_between(t_rel, lateral_error_signed, 0, alpha=0.15, color='blue')
ax_lat.set_xlabel('Time [s]')
ax_lat.set_ylabel('Lateral error [m]')
ax_lat.set_title('Lateral tracking error over time  (+ = left of path, − = right)')
ax_lat.grid(True)
ax_lat.legend()
plt.tight_layout()
save_path_lat = os.path.join(output_dir, f'{base_name}_lateral_error.png')
plt.savefig(save_path_lat, dpi=150)
plt.show()

# ── Plot 3: x / y / yaw vs reference ────────────────────────────────────────
yaw_unwrapped_ts = np.unwrap(yaw)
yawref_unwrapped_ts = np.unwrap(yawref_t)

fig2, axs2 = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

axs2[0].plot(t_rel, x, 'b-', label='x actual')
axs2[0].plot(t_rel, xref_t, 'r--', label='x ref')
axs2[0].set_ylabel('x [m]')
axs2[0].set_title('x vs xref')
axs2[0].grid(True)
axs2[0].legend()

axs2[1].plot(t_rel, y, 'b-', label='y actual')
axs2[1].plot(t_rel, yref_t, 'r--', label='y ref')
axs2[1].set_ylabel('y [m]')
axs2[1].set_title('y vs yref')
axs2[1].grid(True)
axs2[1].legend()

axs2[2].plot(t_rel, yaw_unwrapped_ts, 'b-', label='yaw actual')
axs2[2].plot(t_rel, yawref_unwrapped_ts, 'r--', label='yaw ref')
axs2[2].set_ylabel('yaw [rad]')
axs2[2].set_xlabel('Time [s]')
axs2[2].set_title('yaw vs yawref')
axs2[2].grid(True)
axs2[2].legend()

plt.tight_layout()
save_path_state = os.path.join(output_dir, f'{base_name}_state_vs_ref.png')
plt.savefig(save_path_state, dpi=150)
plt.show()

# ── Print metrics ────────────────────────────────────────────────────────────
avg_speed_cmd = np.mean(speed_cmd)
avg_speed_actual = np.mean(np.abs(v))
total_time = timestamp[-1] - timestamp[0]
yaw_rate_err = yaw_rate_actual - yaw_rate_pred
yaw_rate_rmse_deg = np.degrees(np.sqrt(np.mean(yaw_rate_err ** 2)))
yaw_rate_mae_deg = np.degrees(np.mean(np.abs(yaw_rate_err)))

if np.std(yaw_rate_actual) > 1e-9 and np.std(yaw_rate_pred) > 1e-9:
    yaw_rate_corr = float(np.corrcoef(yaw_rate_actual, yaw_rate_pred)[0, 1])
else:
    yaw_rate_corr = float('nan')

print(f"\n── Tracking metrics (distance to reference path) ──")
print(f"RMSE : {rmse:.4f} m")
print(f"Mean : {mean_error:.4f} m")
print(f"Max  : {max_error:.4f} m")
print(f"Min  : {min_error:.4f} m")
print(f"Lateral RMSE (signed-line diagnostic): {np.sqrt(np.mean(lateral_error ** 2)):.4f} m")
print(f"Duration     : {total_time:.2f} s")
print(f"Actual span  : {actual_span:.3f} m")
print(f"Fit direction: ({fit_dir_x:.3f}, {fit_dir_y:.3f})  yaw={math.degrees(math.atan2(fit_dir_y, fit_dir_x)):.1f} deg")
print(f"Avg commanded speed        : {avg_speed_cmd:.4f} m/s")
print(f"Avg actual speed (OptiTrack): {avg_speed_actual:.4f} m/s")
print(f"Yaw-rate RMSE (actual vs pred from steer): {yaw_rate_rmse_deg:.2f} deg/s")
print(f"Yaw-rate MAE  (actual vs pred from steer): {yaw_rate_mae_deg:.2f} deg/s")
print(f"Yaw-rate correlation: {yaw_rate_corr:.3f}")
print(f"Saved path plot  : {save_path}")
print(f"Saved lateral err: {save_path_lat}")
print(f"Saved state plot : {save_path_state}")