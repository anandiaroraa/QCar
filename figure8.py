#!/usr/bin/env python3
import time
import math
from pal.products.qcar import QCar

#trajectory
def lemniscate_xy(A, s):
    """
        x = A * sin(s)
        y = A * sin(s) * cos(s) = (A/2)*sin(2s)
    """
    x = A * math.sin(s)
    y = A * math.sin(s) * math.cos(s)
    return x, y
#clamp
def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def stop(car):
    car.write(0.0, 0.0)

#control inputs
def run_lemniscate_once(car,
                        A=0.6,          
                        v_scale=0.10,   #speed scaling
                        s_period=2*math.pi,  #lemniscate traversal for one cycle
                        duration=18.0,  # total time to complete once
                        w_max=0.9,      # safety clamp
                        dt=0.05):
    """
        xdot = v cos(psi)
        ydot = v sin(psi)
        psidot = w
    For a parametric curve (x(t), y(t)):
        v = sqrt(xdot^2 + ydot^2)
        w = (xdot*yddot - ydot*xddot) / (xdot^2 + ydot^2)
    """

    # We move s from 0 -> 2pi over 'duration' seconds (one ∞)
    # s(t) = (2pi/duration) * t
    sdot = s_period / duration

    # small step in s-space for numerical derivatives
    eps = 1e-3

    t0 = time.time()
    while True:
        t = time.time() - t0
        if t >= duration:
            break

        s = sdot * t  # current path parameter

        #wr t s
        xm, ym = lemniscate_xy(A, s - eps)
        x0, y0 = lemniscate_xy(A, s)
        xp, yp = lemniscate_xy(A, s + eps)

        dxds = (xp - xm) / (2 * eps)
        dyds = (yp - ym) / (2 * eps)
        d2xds2 = (xp - 2*x0 + xm) / (eps * eps)
        d2yds2 = (yp - 2*y0 + ym) / (eps * eps)

        # derivatives from ds to dt using sdot ---
        xdot = dxds * sdot
        ydot = dyds * sdot
        xddot = d2xds2 * (sdot ** 2)
        yddot = d2yds2 * (sdot ** 2)

        ## --- compute the "raw" (v, w) implied by the trajectory timing ---
        v_raw = math.hypot(xdot, ydot)  # speed from the path
        denom = xdot*xdot + ydot*ydot
        if denom < 1e-9:
            w_raw = 0.0
        else:
            w_raw = (xdot*yddot - ydot*xddot) / denom  
        v = v_scale * v_raw
        w = v_scale * w_raw

        # optional: ensure the car actually moves (QCar often needs a minimum v)
        v_min = 0.05
        if v > 0 and v < v_min:
            # keep direction same, but bump magnitude
            scale_up = v_min / max(v, 1e-9)
            v = v_min
            w = w * scale_up  # keep curvature consistent

        # safety clamp
        w = clamp(w, -w_max, w_max)

        car.write(v, w)
        time.sleep(dt)


def main():
    car = QCar()
    time.sleep(1)

    try:
        print("run once")
        run_lemniscate_once(
            car,
            A=0.5,        #
            v_scale=0.20, # smaller/slower: 0.07–0.12
            duration=20.0,
            w_max=0.9,
            dt=0.05
        )
        print("once complete")
    except KeyboardInterrupt:
        print("\nctrl c")
    finally:
        stop(car)
        print("car stopped")

if __name__ == "__main__":
    main()