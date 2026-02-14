from pal.products.qcar import QCar
import time

car = QCar()
car.open()

speed = 0.0
steering = 0.0

print("WASD Keys")

try:
    while True:
        key = input("Enter command: ")

        if key == 'w':
            speed = 0.1
        elif key == 's':
            speed = 0.1
        elif key == 'a':
            steering = 0.1
        elif key == 'd':
            steering = -0.1
        elif key == 'x':
            speed = 0.0
            steering = 0.0

        car.write(speed, steering)
        time.sleep(0.05)

except KeyboardInterrupt:
    car.write(0, 0)
    car.close()