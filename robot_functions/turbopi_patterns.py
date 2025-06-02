#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/hiwonder/')
import time
import hw_sdk_robot
import tah_sdk_mecanum
import hw_sdk_sonar
import hw_sdk_infrared
import numpy

# ================================================
def box_pattern(wheels) -> int: 
    print("RUNNING BOX PATTERN...")
    wheels.reset_motors()
    time.sleep(0.1)
    wheels.set_velocity(20,0,0.0)
    time.sleep(1)
    wheels.set_velocity(20,90,0.0)
    time.sleep(1)
    wheels.set_velocity(20,180,0.0)
    time.sleep(1)
    wheels.set_velocity(20,270,0.0)
    time.sleep(1)
    wheels.reset_motors()
    return 0
# ================================================

# ================================================
def circle_pattern(wheels) -> int: 
    print("RUNNING CIRCLE PATTERN...")
    timestep = 0.2
    steps    = 101
    speed    = 30
    t,dt = numpy.linspace(0.01,2*numpy.pi,num=steps,endpoint=False,retstep=True)
    vx = -numpy.sin(t)
    vy =  numpy.cos(t)
    angle = numpy.arctan2(vy,vx)
    for i in range(steps):
        if angle[i] < 0 :
            angle[i] += 2*numpy.pi
    angle *= (180.0/numpy.pi)
    
    wheels.reset_motors()
    time.sleep(1)
    for i in range(steps):
        wheels.set_velocity(speed, angle[i], 0.0)
        time.sleep(timestep)
    wheels.reset_motors()
    return 0
# ================================================

# ================================================
def lissajou_pattern(wheels) -> int: 
    print("RUNNING LISSAJOU PATTERN...")
    timestep = 0.2
    steps    = 101
    speed    = 30
    t,dt = numpy.linspace(0.01,2*numpy.pi,num=steps,endpoint=False,retstep=True)
    vx = -numpy.sin(2*t)
    vy =  numpy.cos(t)
    angle = numpy.arctan2(vy,vx)
    for i in range(steps):
        if angle[i] < 0 :
            angle[i] += 2*numpy.pi
    angle *= (180.0/numpy.pi)
    
    wheels.reset_motors()
    time.sleep(1)
    for i in range(steps):
        wheels.set_velocity(speed, angle[i], 0.0)
        time.sleep(timestep)
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
    box_pattern(wheels)
    lissajou_pattern(wheels)
    
    return 0

if __name__ == '__main__':
    sys.exit(main())