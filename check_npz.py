import numpy as np
import matplotlib.pyplot as plt
import math
import glob, os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from MPC_Hardware.qcar_params import MAX_SPEED, MIN_SPEED, MAX_STEER, MAX_DSTEER, MAX_ACCEL, DT, WB, RADIUS, TARGET_SPEED, MAX_TIME, DS
from MPC_Hardware.mpcspeed_steercontrol import T, R, Q, Rd, Qf

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

x           = hist[:, 0]
y           = hist[:, 1]
yaw         = hist[:, 2]
v           = hist[:, 3]
steer       = hist[:, 4]
speed_cmd   = hist[:, 5]
timestamp   = hist[:, 6]

theta_start = hist[0, 2]

# circle center
radius = float(d['radius']) if 'radius' in d else RADIUS
target_speed = float(d['target_speed']) if 'target_speed' in d else TARGET_SPEED
# center_x = x[0] + radius * math.sin(theta_start)
# center_y = y[0] - radius * math.cos(theta_start)
#waypoint correction
center_x = float(d['center_x'])
center_y = float(d['center_y'])

theta_circle = np.linspace(0, 2*math.pi, 200)
ref_x = center_x + radius * np.cos(theta_circle)
ref_y = center_y + radius * np.sin(theta_circle)

# logged MPC waypoints, if available
if 'reference_x' in d and 'reference_y' in d:
    wp_x = np.asarray(d['reference_x']).reshape(-1)
    wp_y = np.asarray(d['reference_y']).reshape(-1)
else:
    wp_x = ref_x
    wp_y = ref_y

# actual trajectory waypoints (downsample to match reference waypoint count)
n_ref_wp = len(wp_x)
actual_wp_step = max(1, len(x) // n_ref_wp)
actual_wp_x = x[::actual_wp_step]
actual_wp_y = y[::actual_wp_step]

# Calculate tracking error as distance from actual trajectory to nearest reference waypoint
dists = np.sqrt((x[:, np.newaxis] - wp_x[np.newaxis, :]) ** 2 + (y[:, np.newaxis] - wp_y[np.newaxis, :]) ** 2)
tracking_error = np.min(dists, axis=1)

# sample timing (used for steer-rate calculation)
dt_samples = np.diff(timestamp)


# steer-following check: compare measured yaw-rate vs yaw-rate predicted from steer command
yaw_unwrapped = np.unwrap(yaw)
yaw_rate_actual = np.divide(
    np.diff(yaw_unwrapped),
    dt_samples,
    out=np.zeros_like(dt_samples),
    where=dt_samples > 0,
)
yaw_rate_pred = (v[:-1] / WB) * np.tan(steer[:-1])

# plt
fig, axs = plt.subplots(1, 4, figsize=(24, 5))

# path tracking (left)
axs.plot(ref_y, ref_x, 'r--', label='Reference')
axs.plot(wp_y, wp_x, 'y*', markersize=6, label='Reference waypoints')
axs.plot(y, x, 'b-', label='Actual')
axs.plot(actual_wp_y, actual_wp_x, 'c*', markersize=5, label='Actual waypoints')
axs.plot(y[0], x[0], 'go', markersize=10, label='Start')
axs.plot(y[-1], x[-1], 'rs', markersize=10, label='End')
axs.set_title(f'Path Tracking (radius={radius}m)\nRMSE={np.sqrt(np.mean(tracking_error**2)):.3f}m  Mean={tracking_error.mean():.3f}m  Max={tracking_error.max():.3f}m')
axs.legend()
axs.axis('equal')
axs.grid(True)

# # speed over time (right)
t_rel = timestamp - timestamp[0]
axs[1].plot(t_rel, v, 'b-', label='Actual speed')
axs[1].plot(t_rel, speed_cmd, 'r-', label='Commanded speed')
axs[1].axhline(y=MAX_SPEED, color='g', linestyle='-', label='MAX_SPEED')
axs[1].axhline(y=MIN_SPEED, color='orange', linestyle='-', label='MIN_SPEED')
axs[1].axhline(y=TARGET_SPEED, color='purple', linestyle='-', label='Target speed')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Speed (m/s)')
axs[1].set_title('Speed Profile')
axs[1].legend()
axs[1].grid(True)

# # steering angle over time (far right)
steer_deg = np.degrees(steer)
max_steer_deg = np.degrees(MAX_STEER)
axs[2].plot(t_rel, steer_deg, 'm-', label='Steer angle')
axs[2].axhline(y=max_steer_deg, color='k', linestyle='--', label='+MAX_STEER')
axs[2].axhline(y=-max_steer_deg, color='k', linestyle='--', label='-MAX_STEER')
axs[2].set_xlabel('Time (s)')
axs[2].set_ylabel('Steering Angle (deg)')
axs[2].set_title('Steering Profile')
axs[2].legend()
axs[2].grid(True)

# # steering rate over time
dsteer = np.divide(np.diff(steer), dt_samples, out=np.zeros_like(dt_samples), where=dt_samples > 0)
dsteer_deg = np.degrees(dsteer)
max_dsteer_deg = np.degrees(MAX_DSTEER)
axs[3].plot(t_rel[1:], np.degrees(yaw_rate_actual), 'c-', label='Yaw rate (measured)')
axs[3].plot(t_rel[1:], np.degrees(yaw_rate_pred), 'm--', label='Yaw rate (pred from steer)')
axs[3].plot(t_rel[1:], dsteer_deg, color='0.6', linestyle=':', label='Steer rate')
axs[3].axhline(y=max_dsteer_deg, color='k', linestyle='--', label='+MAX_DSTEER')
axs[3].axhline(y=-max_dsteer_deg, color='k', linestyle='--', label='-MAX_DSTEER')
axs[3].set_xlabel('Time (s)')
axs[3].set_ylabel('Rate (deg/s)')
axs[3].set_title('Yaw-Rate Consistency Check')
axs[3].legend()
axs[3].grid(True)


plt.subplots_adjust(left=0.22)
plt.tight_layout()

# save plots in tuning_circle folder with same base name as npz
tuning_dir = os.path.join(script_dir, 'Tuning_circle')
os.makedirs(tuning_dir, exist_ok=True)
base_name = os.path.splitext(os.path.basename(latest))[0]
save_path = os.path.join(tuning_dir, f'{base_name}_plot.png')
plt.savefig(save_path)
plt.show()

# Print metrics once
rmse = np.sqrt(np.mean(tracking_error**2))
mean_error = tracking_error.mean()
max_error = tracking_error.max()
min_error = tracking_error.min()
avg_speed_cmd = np.mean(speed_cmd)
avg_speed_actual = np.mean(np.abs(v))
total_time = timestamp[-1] - timestamp[0]
yaw_rate_err = yaw_rate_actual - yaw_rate_pred
yaw_rate_rmse_deg = np.degrees(np.sqrt(np.mean(yaw_rate_err**2)))
yaw_rate_mae_deg = np.degrees(np.mean(np.abs(yaw_rate_err)))

if np.std(yaw_rate_actual) > 1e-9 and np.std(yaw_rate_pred) > 1e-9:
    yaw_rate_corr = float(np.corrcoef(yaw_rate_actual, yaw_rate_pred)[0, 1])
else:
    yaw_rate_corr = float('nan')

print(f"RMSE: {rmse:.4f} m")
print(f"Mean: {mean_error:.4f} m")
print(f"Max:  {max_error:.4f} m")
print(f"Min:  {min_error:.4f} m")
print(f"Duration: {total_time:.2f} s")
print(f"Avg commanded speed: {avg_speed_cmd:.4f} m/s")
print(f"Avg actual speed (v from OptiTrack): {avg_speed_actual:.4f} m/s")
print(f"Yaw-rate RMSE (actual vs pred from steer): {yaw_rate_rmse_deg:.2f} deg/s")
print(f"Yaw-rate MAE  (actual vs pred from steer): {yaw_rate_mae_deg:.2f} deg/s")
print(f"Yaw-rate correlation: {yaw_rate_corr:.3f}")
print(f"Saved to {save_path}")