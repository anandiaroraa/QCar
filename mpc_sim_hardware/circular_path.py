import numpy as np
import math
from utils import wrap_angle
#replacement of from PathPlanning.CubicSpline import cubic_spline_planner 

def calc_circle_course(radius=2.0, ds=0.1, center_x=0.0, center_y=0.0, clockwise=False):
    '''
    Circular path generator 
    Returns
    rx, ry, ryaw, rk, s
    '''
    if radius <= 0:
        raise ValueError("radius must be > 0")
    if ds <= 0:
        raise ValueError("ds must be > 0")

    circumference = 2.0 * math.pi * radius
    n_points = max(3, int(circumference / ds) + 1)

    # Parameter around circle
    theta = np.linspace(0.0, 2.0 * math.pi, n_points, endpoint=False)

    # Position
    rx = center_x + radius * np.cos(theta)
    ry = center_y + radius * np.sin(theta)

    # Yaw = tangent direction
    # CCW: theta + pi/2, CW: theta - pi/2
    if clockwise:
        ryaw = theta - math.pi / 2.0
        curvature = -1.0 / radius
    else:
        ryaw = theta + math.pi / 2.0
        curvature = 1.0 / radius

    # Wrap yaw to [-pi, pi)
    ryaw = np.array([wrap_angle(a) for a in ryaw])

    # Curvature constant
    rk = np.full(n_points, curvature)

    # Arc length from start
    s = np.linspace(0.0, circumference, n_points, endpoint=False)

    return rx.tolist(), ry.tolist(), ryaw.tolist(), rk.tolist(), s.tolist()


def demo_circle():
    import matplotlib.pyplot as plt

    radius = 2.0
    ds = 0.1
    cx, cy = 0.0, 0.0

    rx, ry, ryaw, rk, s = calc_circle_course(radius=radius, ds=ds, center_x=cx, center_y=cy, clockwise=False)

    # Plot path
    plt.figure()
    plt.plot(rx, ry, "-r", label="Circle path")
    plt.plot([cx], [cy], "xb", label="Center")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()

    # Plot yaw vs s
    plt.figure()
    plt.plot(s, [np.rad2deg(y) for y in ryaw], "-r", label="yaw")
    plt.grid(True)
    plt.xlabel("arc length s [m]")
    plt.ylabel("yaw [deg]")
    plt.legend()

    # Plot curvature vs s
    plt.figure()
    plt.plot(s, rk, "-r", label="curvature")
    plt.grid(True)
    plt.xlabel("arc length s [m]")
    plt.ylabel("curvature [1/m]")
    plt.legend()

    plt.show()


if __name__ == "__main__":
    # Use this only when you want to visualize the circle
    demo_circle()