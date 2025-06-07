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


# ================================================
def test_motors(wheels) -> int: 
    print("TESTING MOTORS...")
    wheels.reset_motors()
    time.sleep(1)
    wheels.set_velocity(40,0,0.0)
    time.sleep(1)
    wheels.set_velocity(40,180,0.0)
    time.sleep(1)
    wheels.reset_motors()
    return 0
# ================================================

# ================================================
def test_buzzer(robot) -> int: 
    print("TESTING BUZZER...")
    robot.set_buzzer(2400, 0.1, 0.9, 1)
    return 0
# ================================================

# ================================================
def test_battery(robot) -> int: 
    dat = []
    vi = 0
    print("TESTING BATTERY...")
    while True:
        try:
            volt = robot.get_battery()
            if volt is not None:
                dat.insert(vi, volt/1000.)
                vi += 1
                if vi >= 3:
                    vi = 0
                    voltage = (dat[0]+dat[1]+dat[2])/3.0 
                    print('Voltage:','%0.2f' % voltage)
                    break
            time.sleep(0.1)
        except KeyboardInterrupt:
            break
    return 0
# ================================================

# ================================================
def test_servos(robot) -> int: 
    print("TESTING SERVO 1...")
    robot.pwm_servo_set_position(0.3, [[1, 1800]]) 
    time.sleep(0.3)
    robot.pwm_servo_set_position(0.3, [[1, 1500]]) 
    time.sleep(1.5)

    print("TESTING SERVO 2...")
    robot.pwm_servo_set_position(0.3, [[2, 200]]) 
    time.sleep(3.0)
    robot.pwm_servo_set_position(0.3, [[2, 2300]])
    time.sleep(3.0)
    robot.pwm_servo_set_position(0.3, [[2, 1500]]) 
    time.sleep(1.5)
    return 0
# ================================================

# ================================================
def test_board_rgb(robot) -> int: 
    print("TESTING BOARD RGB...")
    robot.set_rgb([[1, 0, 0, 255]])
    time.sleep(1)
    robot.set_rgb([[1, 0, 0, 0]])
    time.sleep(1)
    return 0
# ================================================

# ================================================
def test_sonar(sonar) -> int: 
    print("TESTING SONAR...")
    sonar.setRGBMode(0)
    time.sleep(0.1)
    sonar.setPixelColor(0, (255, 0, 0))
    sonar.setPixelColor(1, (255, 0, 0))
#    sonar.show()
    time.sleep(1)
    sonar.setPixelColor(0, (0, 0, 0))
    sonar.setPixelColor(1, (0, 0, 0))
 #   sonar.show()
    time.sleep(1)
    for i in range(10) :
        time.sleep(0.1)
        print(i, "Distance:", sonar.getDistance(), " mm")
    return 0
# ================================================

# ================================================
def test_infrared(infrared) -> int: 
    print("TESTING INFRARED LINE FOLLOWER...")
    for i in range(10) :
        data = infrared.readData()
        print(i, "Sensor1:", data[0], " Sensor2:", data[1], " Sensor3:", data[2], " Sensor4:", data[3])
        time.sleep(0.5)
    return 0
# ================================================

def main() -> int:

    robot = hw_sdk_robot.Board()
    robot.enable_reception()
    test_battery(robot)
    test_buzzer(robot)
    test_servos(robot)
    test_board_rgb(robot)

    wheels = tah_sdk_mecanum.Mecanum(robot)
    test_motors(wheels)

    sonar = hw_sdk_sonar.Sonar()
    test_sonar(sonar)
    
    infrared = hw_sdk_infrared.Infrared()
    test_infrared(infrared)
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
