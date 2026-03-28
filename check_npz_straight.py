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
    LENGTH,
    TARGET_SPEED,
)

npz_pattern = os.path.join(script_dir, 'hardware_results_test_straight1', '*.npz')
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

# Build a straight-line reference from the start pose heading.
dir_x = math.cos(theta_start)
dir_y = math.sin(theta_start)

start_x = x[0]
start_y = y[0]

dx_from_start = x - start_x
dy_from_start = y - start_y

# Signed along-track distance for each point relative to the start pose.
along_track = dx_from_start * dir_x + dy_from_start * dir_y

# Exact cross-track error to the infinite straight reference line.
# For unit direction [dir_x, dir_y], 2D cross product gives perpendicular distance.
tracking_error = np.abs(dx_from_start * dir_y - dy_from_start * dir_x)

# Create reference line points that span the data range.
line_margin = 0.5
s_min = float(np.min(along_track) - line_margin)
s_max = float(np.max(along_track) + line_margin)
s_ref = np.linspace(s_min, s_max, 200)
ref_x = start_x + s_ref * dir_x
ref_y = start_y + s_ref * dir_y

# Sample timing (used for steer-rate calculation).
dt_samples = np.diff(timestamp)

target_speed = float(d['target_speed']) if 'target_speed' in d else TARGET_SPEED

# Plot
fig, axs = plt.subplots(1, 4, figsize=(24, 5))

# Path tracking
axs[0].plot(ref_y, ref_x, 'r--', label='Reference (Straight)')
axs[0].plot(y, x, 'b-', label='Actual')
axs[0].plot(y[0], x[0], 'go', markersize=10, label='Start')
axs[0].plot(y[-1], x[-1], 'rs', markersize=10, label='End')
axs[0].set_title(
    f'Straight-Line Tracking\n'
    f'RMSE={np.sqrt(np.mean(tracking_error**2)):.3f}m  '
    f'Mean={tracking_error.mean():.3f}m  '
    f'Max={tracking_error.max():.3f}m'
)
axs[0].legend()
axs[0].axis('equal')
axs[0].grid(True)

# Speed profile
t_rel = timestamp - timestamp[0]
axs[1].plot(t_rel, v, 'b-', label='Actual speed')
axs[1].plot(t_rel, speed_cmd, 'r-', label='Commanded speed')
axs[1].axhline(y=MAX_SPEED, color='g', linestyle='-', label='MAX_SPEED')
axs[1].axhline(y=MIN_SPEED, color='orange', linestyle='-', label='MIN_SPEED')
axs[1].axhline(y=target_speed, color='purple', linestyle='-', label='Target speed')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Speed (m/s)')
axs[1].set_title('Speed Profile')
axs[1].legend()
axs[1].grid(True)

# Steering angle profile
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

# Steering rate profile
dsteer = np.divide(
    np.diff(steer),
    dt_samples,
    out=np.zeros_like(dt_samples),
    where=dt_samples > 0,
)
dsteer_deg = np.degrees(dsteer)
max_dsteer_deg = np.degrees(MAX_DSTEER)
axs[3].plot(t_rel[1:], dsteer_deg, 'c-', label='Steer rate')
axs[3].axhline(y=max_dsteer_deg, color='k', linestyle='--', label='+MAX_DSTEER')
axs[3].axhline(y=-max_dsteer_deg, color='k', linestyle='--', label='-MAX_DSTEER')
axs[3].set_xlabel('Time (s)')
axs[3].set_ylabel('Steering Rate (deg/s)')
axs[3].set_title('Steering Rate Profile')
axs[3].legend()
axs[3].grid(True)

plt.tight_layout()

# Save in Tuning_straight folder with npz-based name
output_dir = os.path.join(script_dir, 'Tuning_straight')
os.makedirs(output_dir, exist_ok=True)
base_name = os.path.splitext(os.path.basename(latest))[0]
save_path = os.path.join(output_dir, base_name + '_plot.png')
plt.savefig(save_path)
plt.show()

# Metrics
rmse = np.sqrt(np.mean(tracking_error**2))
mean_error = tracking_error.mean()
max_error = tracking_error.max()
min_error = tracking_error.min()
total_time = timestamp[-1] - timestamp[0]
avg_speed_cmd = np.mean(speed_cmd)
avg_speed_actual = np.mean(np.abs(v))


print(f"RMSE: {rmse:.4f} m")
print(f"Mean: {mean_error:.4f} m")
print(f"Max:  {max_error:.4f} m")
print(f"Min:  {min_error:.4f} m")
print(f"Duration: {total_time:.2f} s")
print(f"Avg commanded speed: {avg_speed_cmd:.4f} m/s")
print(f"Avg actual speed (v from OptiTrack): {avg_speed_actual:.4f} m/s")
print(f"Saved to {save_path}")
