import time
from laser_rangefinder import laser_rangefinder
from protractor import protractor


if __name__=='__main__':
    ptr = protractor()
    lr = laser_rangefinder()
    while True:
        distance = lr.read_sensor_value()
        angle = ptr.read_sensor_value()
        print(distance)
        print(angle)
        time.sleep(1)