import numpy as np
import matplotlib.pyplot as plt
import math
import glob, os

# Load latest file
files = sorted(glob.glob('hardware_results_test_straight1/*.npz'), key=os.path.getmtime)
latest = files[-1]
print("Loading:", latest)

d = np.load(latest, allow_pickle=True)
hist = d['car1_history']

x = hist[:, 0]
y = hist[:, 1]
theta_start = hist[0, 2]

# Reconstruct reference line (same formula as sharedline.py)
line_length = 1.0
n_points = 50
dx = 1.0
dy = 0.0
ref_x = [x[0] + dx * line_length * i / (n_points - 1) for i in range(n_points)]
ref_y = [y[0] + dy * line_length * i / (n_points - 1) for i in range(n_points)]

fig, ax = plt.subplots(figsize=(8, 4))
ax.plot(ref_y, ref_x, 'r--', label='Reference')
ax.plot(y, x, 'b-', label='Actual')
ax.plot(y[0], x[0], 'go', markersize=10, label='Start')
ax.plot(y[-1], x[-1], 'rs', markersize=10, label='End')
ax.set_title('Straight Line Path Tracking')
ax.legend()
ax.axis('equal')
ax.grid(True)

plt.tight_layout()
plt.savefig('path_tracking_straight.png')
plt.show()
print("Saved to path_tracking_straight.png")