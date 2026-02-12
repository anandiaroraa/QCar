#!/usr/bin/env python3
import time
import math
from pal.products.qcar import QCar

def lemniscate_xy(A, t):
    
    x = A * math.sin(t)
    y = A * math.sin(t) * math.cos(t)
    return x, y



def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def stop(car):
    car.write(0.0, 0.0)


def run_figure8(car,
                A=1.0,            # size of the 8 (bigger = wider)
                v=0.1,           # constant forward speed
                t_speed=0.1,      # how fast we move along the curve (bigger = faster)
                lookahead=0.35,   # how far ahead on the path we aim (bigger = smoother)
                k=1.8,            # turning gain (bigger = turns harder)
                w_max=0.7,        # safety limit on turning command
                duration=25.0,
                dt=0.05):

    #progress along the path
    t = 0.0

    #kept a simple estimate of where we are in the (x,y) plane by integrating v
    #approximation, but good enough for drawing a figure-8.
    x_est, y_est = lemniscate_xy(A, t)
    heading = 0.0  # estimated heading angle (rad)

    t0 = time.time()
    while time.time() - t0 < duration:
        #current desired point on the lemniscate
        x_ref, y_ref = lemniscate_xy(A, t)

        #target point ahead on the lemniscate
        x_tar, y_tar = lemniscate_xy(A, t + lookahead)

        #dir angle
        dx = x_tar - x_est
        dy = y_tar - y_est
        target_angle = math.atan2(dy, dx)

        #target-where we are
        err = target_angle - heading

        #wrapping the error
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi

        #turn
        w = k * err
        w = clamp(w, -w_max, w_max)

        
        car.write(v, w)

        #update our simple estimated state
        #heading changes by w*dt, position changes by v along heading
        heading += w * dt
        x_est += v * math.cos(heading) * dt
        y_est += v * math.sin(heading) * dt

        #the prorogress along the curve
        t += t_speed * dt

        time.sleep(dt)


def main():
    car = QCar()
    time.sleep(1)

    try:
        print("Figure 8")
        run_figure8(
            car,
            A=1.0,
            v=0.12,
            t_speed=0.7,
            lookahead=0.45,
            k=2.0,
            w_max=0.7,
            duration=25.0,
            dt=0.05
        )
    finally:
        stop(car)
        print("Finished.")


if __name__ == "__main__":
    main()