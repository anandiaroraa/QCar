#!/usr/bin/env python3
import time
from pal.products.qcar import QCar

#speed v, turn w throttle, steering (pal language)
def hold(car, v, w, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        car.write(v, w)
        time.sleep(0.05)
#to check if the angles are proper of the tyres before the next move that I give
def stop(car):
    car.write(0.0, 0.0)
#experiemnt
def center_steering(car):
    hold(car, 0.0, 0.0, 1.0)
    hold(car, 0.0, 0.15, 0.4)
    hold(car, 0.0, -0.15, 0.4)
    hold(car, 0.0, 0.0, 1.0)

#change the initial tyre angles
def pause(car, wait_time=0.5):
    center_steering(car)
    stop(car)
    time.sleep(wait_time)

def main():
    car = QCar()
    time.sleep(1)

    try:
        pause(car, wait_time=0.5)
        #f
        hold(car, 0.1, 0.0, 4.0)
        pause(car, wait_time=4.0)
        #back
        hold(car, -0.1, 0.0, 4.0)
        pause(car, wait_time=4.0)
        #right
        hold(car, 0.1, -0.4, 4.0)
        pause(car, wait_time=4.0)
        #left
        hold(car, 0.1, 0.4, 4.0)
        pause(car, wait_time=4.0)

    finally:
        stop(car)
        hold(car, 0.0, 0.0, 0.5)
        print("Woaahhhh car is stopped.")

if __name__ == "__main__":
    main()
