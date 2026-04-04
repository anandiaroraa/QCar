import numpy as np
import math
#from .utils import angle_mod
from .qcar_params import RADIUS, LENGTH, DS
import matplotlib.pyplot as plt

def plot_trajectory(rx, ry, ryaw=None, title="Reference Trajectory", show_points=True):
    """
    Quick visual check for generated trajectory.

    Args:
        rx, ry: trajectory x/y points
        ryaw: optional yaw array for heading arrows
        title: plot title
        show_points: scatter all points if True
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib is not installed; skipping trajectory plot.")
        return

    rx_arr = np.asarray(rx)
    ry_arr = np.asarray(ry)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(rx_arr, ry_arr, "b-", linewidth=2.0, label="reference")
    if show_points:
        ax.scatter(rx_arr, ry_arr, s=14, c="tab:orange", label="waypoints")

    if ryaw is not None and len(ryaw) == len(rx_arr):
        step = max(1, len(rx_arr) // 25)
        yaw_arr = np.asarray(ryaw)
        ax.quiver(
            rx_arr[::step],
            ry_arr[::step],
            np.cos(yaw_arr[::step]),
            np.sin(yaw_arr[::step]),
            angles="xy",
            scale_units="xy",
            scale=8.0,
            width=0.003,
            color="tab:green",
            label="heading"
        )

    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True)
    ax.axis("equal")
    ax.legend(loc="best")
    plt.tight_layout()
    plt.show()

def calc_straight_course(length=LENGTH, ds=DS, start_x=0.0, start_y=0.0, angle=0.0):
    """
    Straight line path generator.
    
    Args:
        length: Length of straight line (meters, default from qcar_params.LENGTH)
        ds: Arc length spacing (meters, default from qcar_params.DS)
        start_x: Starting x position
        start_y: Starting y position
        angle: Heading angle (radians)
    
    Returns:
        rx, ry, ryaw, rk, s: Lists of x, y, yaw, curvature, arc length
    """
    if length <= 0:
        raise ValueError("length must be > 0")
    if ds <= 0:
        raise ValueError("ds must be > 0")
    
    n_points = max(3, int(length / ds) + 1)
    
    # Distance along line
    s = np.linspace(0.0, length, n_points)
    
    # Position along straight line
    rx = start_x + s * np.cos(angle)
    ry = start_y + s * np.sin(angle)
    
    # Yaw constant (heading angle)
    ryaw = np.full(n_points, angle_mod(angle))
    
    # Curvature = 0 (straight line)
    rk = np.zeros(n_points)
    
    return rx.tolist(), ry.tolist(), ryaw.tolist(), rk.tolist(), s.tolist()


def calc_circle_course(radius=RADIUS, ds=DS, center_x=0.0, center_y=0.0, clockwise=False):
    """
    Circular path generator.
    
    Args:
        radius: Circle radius (meters, default from qcar_params.RADIUS)
        ds: Arc length spacing (meters, default from qcar_params.DS)
        center_x: Circle center x position
        center_y: Circle center y position
        clockwise: Boolean for clockwise direction
    
    Returns:
        rx, ry, ryaw, rk, s: Lists of x, y, yaw, curvature, arc length
    """
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
        ryaw = theta + math.pi / 2.0
        curvature = -1.0 / radius
    else:
        ryaw = theta - math.pi / 2.0
        curvature = 1.0 / radius

    # Wrap yaw to [-pi, pi)
    # ryaw = np.array([angle_mod(a) for a in ryaw])
    # (angle + np.pi) % (2 * np.pi) - np.pi
    ryaw = (ryaw + np.pi) % (2 * np.pi) - np.pi
    assert abs(ryaw).max() <= math.pi and abs(ryaw).min() >= -math.pi, "yaw not wrapped to [-pi, pi)"
    # Curvature constant
    rk = np.full(n_points, curvature)

    # Arc length from start
    s = np.linspace(0.0, circumference, n_points, endpoint=False)

    return rx.tolist(), ry.tolist(), ryaw.tolist(), rk.tolist(), s.tolist()


def get_trajectory(trajectory_type="circle", **kwargs):
    """
    Unified trajectory selector.
    
    Args:
        trajectory_type: "circle" or "straight"
        **kwargs: trajectory-specific parameters
        
    Circle parameters:
        - radius: Circle radius (default from qcar_params.RADIUS)
        - ds: Arc length spacing (default from qcar_params.DS)
        - center_x: Circle center x (default 0.0)
        - center_y: Circle center y (default 0.0)
        - clockwise: Boolean (default False)
    
    Straight parameters:
        - length: Line length (default from qcar_params.LENGTH)
        - ds: Arc length spacing (default from qcar_params.DS)
        - start_x: Start x (default 0.0)
        - start_y: Start y (default 0.0)
        - angle: Heading angle in radians (default 0.0)
    
    Returns:
        rx, ry, ryaw, rk, s: Lists of x, y, yaw, curvature, arc length
    """
    if trajectory_type.lower() == "circle":
        rx, ry, ryaw, rk, s = calc_circle_course(
            radius=kwargs.get("radius", RADIUS),
            ds=kwargs.get("ds", DS),
            center_x=kwargs.get("center_x", 0.0),
            center_y=kwargs.get("center_y", 0.0),
            clockwise=kwargs.get("clockwise", False)
        )
    elif trajectory_type.lower() == "straight":
        rx, ry, ryaw, rk, s = calc_straight_course(
            length=kwargs.get("length", LENGTH),
            ds=kwargs.get("ds", DS),
            start_x=kwargs.get("start_x", 0.0),
            start_y=kwargs.get("start_y", 0.0),
            angle=kwargs.get("angle", 0.0)
        )
    else:
        raise ValueError(f"Unknown trajectory_type: {trajectory_type}. Use 'circle' or 'straight'")

    if kwargs.get("debug_plot", False):
        plot_title = kwargs.get("plot_title", f"{trajectory_type.capitalize()} trajectory")
        plot_trajectory(rx, ry, ryaw=ryaw, title=plot_title, show_points=True)

    return rx, ry, ryaw, rk, s

if __name__ == "__main__":
    # Example usage
    circle_rx, circle_ry, circle_ryaw, circle_rk, circle_s = get_trajectory(
        trajectory_type="circle",
        radius=RADIUS,
        ds=DS,
        center_x=0.0,
        center_y=0.0,
        clockwise=False,
        debug_plot=True,
        plot_title="Generated Circle Trajectory"
    )

    straight_rx, straight_ry, straight_ryaw, straight_rk, straight_s = get_trajectory(
        trajectory_type="straight",
        length=LENGTH,
        ds=DS,
        start_x=0.0,
        start_y=0.0,
        angle=math.radians(30),  # 30 degree heading
        debug_plot=True,
        plot_title="Generated Straight Trajectory"
    )