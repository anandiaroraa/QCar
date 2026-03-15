import numpy as np
import matplotlib.pyplot as plt
import math
import glob, os
from .qcar_params import MAX_SPEED

#latest file loaded
files = sorted(glob.glob('hardware_results_test1/*.npz'), key=os.path.getmtime)
latest = files[-1]
print("Loading:", latest)

d = np.load(latest, allow_pickle=True)

#keys
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

#circle center
#radius = 1.0  # match shared2.py
radius = float(d['radius']) if 'radius' in d else 1.0
target_speed = float(d['target_speed']) if 'target_speed' in d else MAX_SPEED
center_x = x[0] + radius * math.sin(theta_start)
center_y = y[0] - radius * math.cos(theta_start)


theta_circle = np.linspace(0, 2*math.pi, 200)
ref_x = center_x + radius * np.cos(theta_circle)
ref_y = center_y + radius * np.sin(theta_circle)

#tracking error
dist_from_center = np.sqrt((x - center_x)**2 + (y - center_y)**2)
tracking_error = np.abs(dist_from_center - radius)

#speed from pos
dt = 0.1
dx = np.diff(x)
dy = np.diff(y)
speed_est = np.sqrt(dx**2 + dy**2) / dt

#metrics
print(f"\n--- Metrics ---")
print(f"Points recorded : {len(x)}")
print(f"Duration        : {timestamp[-1] - timestamp[0]:.2f} s")
print(f"Mean error      : {tracking_error.mean():.4f} m")
print(f"RMSE            : {np.sqrt(np.mean(tracking_error**2)):.4f} m")
print(f"Max error       : {tracking_error.max():.4f} m")
print(f"Std error       : {tracking_error.std():.4f} m")
print(f"Mean speed (est): {speed_est.mean():.3f} m/s")
print(f"Max speed  (est): {speed_est.max():.3f} m/s")
print(f"Min speed  (est): {speed_est.min():.3f} m/s")
print(f"Mean speed (cmd): {speed_cmd.mean():.3f} m/s")

#plt
fig, axs = plt.subplots(1, 2, figsize=(12, 5))

#path tracking (left)
axs[0].plot(ref_y, ref_x, 'r--', label='Reference')
axs[0].plot(y, x, 'b-', label='Actual')
axs[0].plot(y[0], x[0], 'go', markersize=10, label='Start')
axs[0].plot(y[-1], x[-1], 'rs', markersize=10, label='End')
axs[0].set_title(f'Path Tracking (radius={radius}m)\nRMSE={np.sqrt(np.mean(tracking_error**2)):.3f}m  Mean={tracking_error.mean():.3f}m  Max={tracking_error.max():.3f}m')
axs[0].legend()
axs[0].axis('equal')
axs[0].grid(True)

#speed over time (right)
t_rel = timestamp - timestamp[0]
axs[1].plot(t_rel[1:], speed_est, 'b-', label='Estimated speed')
axs[1].plot(t_rel, speed_cmd, 'r--', label='Commanded speed')
axs[1].axhline(y=0.20, color='g', linestyle=':', label='MAX_SPEED')
axs[1].axhline(y=0.15, color='orange', linestyle=':', label='MIN_SPEED')
axs[1].axhline(y=target_speed, color='purple', linestyle=':', label='Target speed')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Speed (m/s)')
axs[1].set_title('Speed Profile')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()

#same name as npz
base_name = os.path.splitext(latest)[0]
save_path = base_name + '_plot.png'
plt.savefig(save_path)
plt.show()
print(f"\nSaved to {save_path}")