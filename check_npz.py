import numpy as np
import matplotlib.pyplot as plt
import math

d = np.load('hardware_results_test1/run_000_t_1772843924.05159.npz', allow_pickle=True)
hist = d['car1_history']

x    = hist[:, 0]
y    = hist[:, 1]
yaw  = hist[:, 2]
v    = hist[:, 3]
steer = hist[:, 4]
speed = hist[:, 5]

# Generate reference circle (same as shared.py)
radius = 1.5
center_x = hist[0, 0]
center_y = hist[0, 1]
theta = np.linspace(0, 2*math.pi, 200)
ref_x = center_x + radius * np.cos(theta)
ref_y = center_y + radius * np.sin(theta)

fig, axs = plt.subplots(2, 2, figsize=(12, 8))

# Plot 1: Path tracking
axs[0, 0].plot(ref_x, ref_y, 'r--', label='Reference')
axs[0, 0].plot(x, y, 'b-', label='Actual')
axs[0, 0].plot(x[0], y[0], 'go', markersize=10, label='Start')
axs[0, 0].set_title('Path Tracking')
axs[0, 0].legend()
axs[0, 0].axis('equal')
axs[0, 0].grid(True)

# Plot 2: Speed over time
t = hist[:, 6] - hist[0, 6]
axs[0, 1].plot(t, v, label='Estimated v')
axs[0, 1].plot(t, speed, label='Commanded speed')
axs[0, 1].set_title('Speed vs Time')
axs[0, 1].set_xlabel('Time (s)')
axs[0, 1].legend()
axs[0, 1].grid(True)

# Plot 3: Steering over time
axs[1, 0].plot(t, np.rad2deg(steer))
axs[1, 0].set_title('Steering Angle vs Time')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].set_ylabel('Degrees')
axs[1, 0].grid(True)

# Plot 4: Yaw over time
axs[1, 1].plot(t, np.rad2deg(yaw))
axs[1, 1].set_title('Yaw vs Time')
axs[1, 1].set_xlabel('Time (s)')
axs[1, 1].set_ylabel('Degrees')
axs[1, 1].grid(True)

plt.tight_layout()
plt.savefig('path_tracking.png')
plt.show()
print("Saved to path_tracking.png")