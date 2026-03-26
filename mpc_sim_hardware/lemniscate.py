import numpy as np
import matplotlib.pyplot as plt

def generate_lemniscate(a=2.0, num_points=500):

    t = np.linspace(0, 2 * np.pi, num_points)

    x = (a * np.cos(t)) / (1 + np.sin(t)**2)
    y = (a * np.sin(t) * np.cos(t)) / (1 + np.sin(t)**2)

    return x, y


def compute_yaw(cx, cy):
    
    dx = np.gradient(cx)
    dy = np.gradient(cy)

    yaw = np.arctan2(dy, dx)
    return yaw


def compute_curvature(cx, cy):
    
    dx = np.gradient(cx)
    dy = np.gradient(cy)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)

    k = (dx * ddy - dy * ddx) / ((dx**2 + dy**2)**1.5 + 1e-6)
    return k


if __name__ == "__main__":

    cx, cy = generate_lemniscate(a=2.0)

    cyaw = compute_yaw(cx, cy)
    ck = compute_curvature(cx, cy)

    plt.figure()
    plt.plot(cx, cy)
    plt.axis("equal")
    plt.title("Lemniscate Path")
    plt.show()