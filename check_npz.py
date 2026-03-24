import numpy as np
import matplotlib.pyplot as plt
import math
import glob, os
import sys
sys.path.insert(0, '/home/anandiarora/QCar')
from MPC_Hardware.qcar_params import MAX_SPEED, MIN_SPEED, MAX_STEER, TARGET_SPEED, RADIUS
from MPC_Hardware.mpcspeed_steercontrol2 import T, Q, Rd, R

# latest file loaded
files = sorted(glob.glob('hardware_results_test1/*.npz'), key=os.path.getmtime)
latest = files[-1]
print("Loading:", latest)

d = np.load(latest, allow_pickle=True)

# keys
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
center_x = x[0] + radius * math.sin(theta_start)
center_y = y[0] - radius * math.cos(theta_start)

theta_circle = np.linspace(0, 2*math.pi, 200)
ref_x = center_x + radius * np.cos(theta_circle)
ref_y = center_y + radius * np.sin(theta_circle)

# tracking error (nearest reference-point distance, same RMSE style as MPC sim)
tracking_error = np.array([
    np.min(np.hypot(ref_x - xi, ref_y - yi))
    for xi, yi in zip(x, y)
])

# speed from pos
dt = 0.1
dx = np.diff(x)
dy = np.diff(y)
speed_est = np.sqrt(dx**2 + dy**2) / dt
speed_est = np.clip(speed_est, 0.0, MAX_SPEED * 3)  # clip for display

# plt
fig, axs = plt.subplots(1, 2, figsize=(12, 5))

# path tracking (left)
axs[0].plot(ref_y, ref_x, 'r--', label='Reference')
axs[0].plot(y, x, 'b-', label='Actual')
axs[0].plot(y[0], x[0], 'go', markersize=10, label='Start')
axs[0].plot(y[-1], x[-1], 'rs', markersize=10, label='End')
axs[0].set_title(f'Path Tracking (radius={radius}m)\nRMSE={np.sqrt(np.mean(tracking_error**2)):.3f}m  Mean={tracking_error.mean():.3f}m  Max={tracking_error.max():.3f}m')
axs[0].legend()
axs[0].axis('equal')
axs[0].grid(True)

# speed over time (right)
t_rel = timestamp - timestamp[0]
axs[1].plot(t_rel[1:], speed_est, 'b-', label='Estimated speed')
axs[1].plot(t_rel, speed_cmd, 'r--', label='Commanded speed')
axs[1].axhline(y=MAX_SPEED, color='g', linestyle=':', label='MAX_SPEED')
axs[1].axhline(y=MIN_SPEED, color='orange', linestyle=':', label='MIN_SPEED')
axs[1].axhline(y=target_speed, color='purple', linestyle=':', label='Target speed')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Speed (m/s)')
axs[1].set_title('Speed Profile')
axs[1].legend()
axs[1].grid(True)

params_text = (
    f"RMSE: {np.sqrt(np.mean(tracking_error**2)):.4f} m\n"
    
)

fig.text(0.02, 0.5, params_text, fontsize=8,
         verticalalignment='center',
         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

plt.subplots_adjust(left=0.22)
plt.tight_layout()

# same name as npz
base_name = os.path.splitext(latest)[0]
save_path = base_name + '_plot.png'
plt.savefig(save_path)
plt.show()

# Print metrics once
rmse = np.sqrt(np.mean(tracking_error**2))
max_error = tracking_error.max()
min_error = tracking_error.min()
total_time = timestamp[-1] - timestamp[0]
print(f"\n--- Tracking Error Metrics ---")
print(f"RMSE: {rmse:.4f} m")
print(f"Max:  {max_error:.4f} m")
print(f"Min:  {min_error:.4f} m")
print(f"Duration: {total_time:.2f} s")
print(f"Saved to {save_path}")