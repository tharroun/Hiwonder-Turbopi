#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/hiwonder/Github/Hiwonder-Turbopi')
sys.path.append('/home/hiwonder/Github/Hiwonder-Turbopi/sdk')
import time
import hw_sdk_robot
import tah_sdk_mecanum
import hw_sdk_sonar
import hw_sdk_infrared
import numpy

# ================================================
def speed_test(wheels) -> int: 
    print("RUNNING SPEED TEST...")
    speed = numpy.linspace(60, 0, num=61, endpoint=True, retstep=False)
    wheels.reset_motors()
    for s in speed:
        print(s)
        wheels.set_velocity(s,0,0.0)
        time.sleep(0.05)
    for s in reversed(speed):
        print(s)
        wheels.set_velocity(s,0,0.0)
        time.sleep(0.05)
    wheels.reset_motors()
    return 0
# ================================================

# ================================================
def buzzer(robot) -> int: 
    print("TESTING BUZZER...")
    robot.set_buzzer(2400, 0.1, 0.9, 1)
    return 0
# ================================================

# ================================================
def test_battery(robot) -> int: 
    data    = []
    samples = 10
    i       = 0
    print("TESTING BATTERY...")
    while True:
        try:
            volt = robot.get_battery()
            if volt is not None:
                data.append(volt/1000.)
                i += 1
                if i >= samples:
                    voltage = sum(data) / float(samples) 
                    print('Voltage:','%0.2f' % voltage)
                    break
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    return 0
# ================================================


def main() -> int:

    robot = hw_sdk_robot.Board()
    robot.enable_reception()
    test_battery(robot)
    buzzer(robot)
    
    wheels = tah_sdk_mecanum.Mecanum(robot)
    speed_test(wheels)
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
