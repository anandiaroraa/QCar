# create a test_waypoints.py file
import math
import numpy as np
from .MPC_Hardware.trajectory import calc_circle_course
from .MPC_Hardware.mpcspeed_steercontrol import calc_speed_profile, smooth_yaw
from .MPC_Hardware.qcar_params import MAX_SPEED, DS, RADIUS

# simulate car start pose
car_x, car_y, car_theta = 0.249, 0.006, -0.004
radius = RADIUS

center_x = car_x + radius * math.sin(car_theta)
center_y = car_y - radius * math.cos(car_theta)

cx, cy, cyaw, ck, _ = calc_circle_course(
    radius=radius, ds=DS,
    center_x=center_x, center_y=center_y,
    clockwise=False
)
sp = calc_speed_profile(cx, cy, cyaw, MAX_SPEED)
cyaw = smooth_yaw(cyaw)

print(f"Center: ({center_x:.3f}, {center_y:.3f})")
print(f"n_points: {len(cx)}")
print(f"First waypoint: ({cx[0]:.3f}, {cy[0]:.3f})")
print(f"Speed profile range: {min(sp):.3f} to {max(sp):.3f}")

# verify car is on circle
dist = math.hypot(car_x - center_x, car_y - center_y)
print(f"Car distance from center: {dist:.3f}m (should be {radius}m)")