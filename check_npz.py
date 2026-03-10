import numpy as np
import matplotlib.pyplot as plt
import math
import glob, os

files = glob.glob('hardware_results/*.npz')
latest = max(files, key=os.path.getmtime)
d = np.load(latest, allow_pickle=True)
print("Loading:", latest)

#d = np.load('hardware_results_test1/test5.npz', allow_pickle=True)
hist = d['car1_history']

np.set_printoptions(precision=4, suppress=True)
print(hist)

x     = hist[:, 0]
y     = hist[:, 1]
theta_start = hist[0, 2]  # yaw at start

# Reconstruct the actual circle center using the same formula as shared2.py
radius = 1.0
center_x = x[0] + radius * math.sin(theta_start)
center_y = y[0] - radius * math.cos(theta_start)

theta = np.linspace(0, 2*math.pi, 200)
ref_x = center_x + radius * np.cos(theta)
ref_y = center_y + radius * np.sin(theta)

fig, ax = plt.subplots(figsize=(6, 6))
ax.plot(ref_y, ref_x, 'r--', label='Reference')
ax.plot(y, x, 'b-', label='Actual')
ax.plot(y[0], x[0], 'go', markersize=10, label='Start')
ax.set_title('Path Tracking')
ax.legend()
ax.axis('equal')
ax.grid(True)

plt.tight_layout()
plt.savefig('path_tracking.png')
plt.show()