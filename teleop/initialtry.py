#!/usr/bin/env python3
import time
from pal.products.qcar import QCar

def hold(car, v, w, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        car.write(v, w)
        time.sleep(0.05)  
def stop(car):
    car.write(0.0, 0.0)
def main():
    car = QCar()
    time.sleep(1)
    v = 0.1
    w = 0.8
    t_left = 5.0
    t_right = 6.0
    try:
        # left loop
        hold(car, v, +w, t_left)
        # right loop
        hold(car, v, -w, t_right)
        #f
        #hold(car, 0.1, 0.0, 4.0)
        # stop
        #stop(car)
        #time.sleep(4)
        #b
        #hold(car, -0.1, 0.0, 4.0)
        #right
        #hold(car, 0.1, -0.4, 4.0)
        #left
        #hold(car, 0.1, 0.4, 4.0)
    finally:
        stop(car)
        print("Works!")
if __name__ == "__main__":
    main()
