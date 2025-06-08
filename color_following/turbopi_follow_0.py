#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/hiwonder/Github/Hiwonder-Turbopi')
sys.path.append('/home/hiwonder/Github/Hiwonder-Turbopi/sdk')
import time
import threading
from queue import Queue
import yaml
import numpy
import mypid
import hw_sdk_robot
import hw_sdk_sonar
import tah_sdk_mecanum
import cv2

KILL_THREAD = object() 
LOST_OBJECT = object()

# -------------
# LOAD THE COLOR DEFINITIONS
calbiration_filename = '/home/hiwonder/Github/Hiwonder-Turbopi/color_calibration/calibration.yaml' 
with open(calbiration_filename,'r') as file:
    color_ranges = yaml.safe_load(file)
COLOR_MIN = numpy.array(color_ranges['green']['min'],numpy.uint8)
COLOR_MAX = numpy.array(color_ranges['green']['max'],numpy.uint8)

small_kernel   = numpy.ones((3, 3), numpy.uint8)
medium_kernel  = numpy.ones((6, 6), numpy.uint8)
large_kernel   = numpy.ones((9, 9), numpy.uint8)

servo2_center = 1500 # x (horizontal) direction
servo1_center = 1500 # y (vertical) direction
set_radius    = 40.0

# ==============================================================
def robot_servo(qs,robot) -> int:

    sx = mypid.mypid(0.15,0.0,0.005)
    sy = mypid.mypid(0.2,0.0,0.005)

    servo_x_position = servo2_center
    servo_y_position = servo1_center

    (width,height) = qs.get()
    qs.task_done()
    assert (width,height)==(640,480) , print("First data in servo queue is not the width/height.")
    cX = width/2.0
    cY = height/2.0

    while True:
        data = qs.get()
        # ----------------------------------------------------------
        # THE TASK
        if data is KILL_THREAD:
            robot.pwm_servo_set_position(0.3, [[1, servo1_center]])
            robot.pwm_servo_set_position(0.3, [[2, servo2_center]])
            qs.task_done()
            break
        elif data is LOST_OBJECT:
            robot.pwm_servo_set_position(0.3, [[1, servo1_center]])
            robot.pwm_servo_set_position(0.3, [[2, servo2_center]])
        else :
            move_x = sx.pid(cX, data[0], data[3])
            new_x_position = int(servo_x_position + move_x)
            if new_x_position < 1000 or new_x_position > 2000: 
                #position_x = int(servo_x - control/4.0) 
                new_x_position = servo_x_position
            # ----------------------------------------------------------
            move_y = sy.pid(cY, data[1], data[3])
            new_y_position = int(servo_y_position - move_y)
            if new_y_position < 1100 or new_y_position > 1900 : new_y_position = servo_y_position
            # ----------------------------------------------------------
            robot.pwm_servo_set_position(0.3, [[1, new_y_position],[2, new_x_position]]) 
            servo_x_position = new_x_position
            servo_y_position = new_y_position
        # ----------------------------------------------------------
        qs.task_done()

    return 1 
# ==============================================================

# ==============================================================
def robot_mecanum(qm) -> int:
    mr = mypid.mypid(0.0015,0.0,0.00005)
    mz = mypid.mypid(0.004,0.0,0.0001)

    wheels = tah_sdk_mecanum.Mecanum(robot)

    (width,height) = qm.get()
    qm.task_done()
    assert (width,height)==(640,480) , print("First data in servo queue is not the width/height.")
    cX = width/2.0
    cY = height/2.0

    while True:
        data = qm.get()
        # ----------------------------------------------------------
        # THE TASK
        if data is KILL_THREAD:
            wheels.reset_motors()
            qm.task_done()
            break
        elif data is LOST_OBJECT:
            wheels.reset_motors()
        else :
            # ----------------------------------------------------------
            if data[0] < 200 :
                control = numpy.fabs(mr.pid(200,data[0],data[3]))
                control = 0.12 if control > 0.12 else control
                wheels.set_velocity(0,90,-0.2)
                time.sleep(control)
                wheels.reset_motors()
            if data[0] > 440:
                control = numpy.fabs(mr.pid(440,data[0],data[3]))
                control = 0.12 if control > 0.12 else control
                wheels.set_velocity(0,90,0.2)
                time.sleep(control)
                wheels.reset_motors()
            # ----------------------------------------------------------
            if numpy.fabs(data[2] - set_radius) > 10:
                control = mz.pid(set_radius,data[2],data[3])
                if control < 0 :
                    control = 0.12 if numpy.fabs(control) > 0.12 else numpy.fabs(control)
                    wheels.set_velocity(20,180,0.0)
                    time.sleep(control)
                    wheels.reset_motors()
                else:
                    control = 0.12 if control > 0.12 else control
                    wheels.set_velocity(20,0,0.0)
                    time.sleep(control)
                    wheels.reset_motors()
        # ----------------------------------------------------------
        qm.task_done()

    wheels.reset_motors()
    return 1 
# ==============================================================

# ==============================================================
def robot_see(qs,qm) -> int:

    cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
    if not cap.isOpened():
        print("Error opening camera")
        qs.put(KILL_THREAD)
        qm.put(KILL_THREAD)
        return -1
    
    width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH )   
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT ) 
    qs.put((width,height))
    qm.put((width,height))

    t1 = time.perf_counter()
    while True:
        result, frame = cap.read()
        if not result:
            print("Error receiving video frame")
            qs.put(KILL_THREAD)
            qm.put(KILL_THREAD)
            return -1
        
        frame_hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
        color_mask = cv2.inRange(frame_hsv, COLOR_MIN, COLOR_MAX)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, small_kernel, iterations = 1)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, small_kernel,iterations = 1)

        contours,hierarchy = cv2.findContours(color_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if (contours):
            t2 = time.perf_counter()
            if (len(contours)>1) : 
                contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
            # --
            area = cv2.contourArea(contours[0])
            arclength = cv2.arcLength(contours[0],True)
            circularity = 4*numpy.pi*area/(arclength*arclength)
            # NEXT TIME:https://docs.opencv.org/4.11.0/d4/d70/tutorial_hough_circle.html
            # --
            M = cv2.moments(contours[0])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            radius = int(numpy.sqrt(M['m00']/numpy.pi))
            if circularity < 0.7 : 
                qs.put(LOST_OBJECT)
                qm.put(LOST_OBJECT)
            else:
                qs.put((cx,cy,radius,t2-t1))
                qm.put((cx,cy,radius,t2-t1))
            #cv2.circle(frame,(cx,cy),radius,(0,255,0),5)
            cv2.drawContours(frame,contours,0,(0,0,255),5)
            t1 = t2
            # --
        cv2.imshow('frame', frame)

        if cv2.waitKey(10) == ord('q'):
            qs.put(KILL_THREAD)
            qm.put(KILL_THREAD)
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    return 1
# ==============================================================

# ==============================================================
def test_servos(robot) -> int: 
    print("TESTING SERVO 1 UP/DOWN...")
    robot.pwm_servo_set_position(0.3, [[1, servo2_center]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[1, servo2_center+200]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[1, servo2_center-200]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[1, servo2_center]]) 
    time.sleep(0.5)
    
    print("TESTING SERVO 2 LEFT/RIGHT...")
    robot.pwm_servo_set_position(0.3, [[2, servo2_center]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[2, servo2_center+200]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[2, servo2_center-200]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[2, servo2_center]]) 
    time.sleep(0.5)
    return 0
# ==============================================================

# ==============================================================
def test_board_rgb(robot) -> int: 
    print("TESTING BOARD RGB...")
    robot.set_rgb([[1, 0, 0, 0]])
    time.sleep(0.5)
    return 0
# ==============================================================

# ==============================================================
def test_sonar(sonar) -> int: 
    print("TESTING SONAR...")
    sonar.setRGBMode(0)
    time.sleep(0.1)
    sonar.setPixelColor(0, (255, 255, 255))
    sonar.setPixelColor(1, (255, 255, 255))
#    sonar.show()
    return 0
# ==============================================================

# ==============================================================
if __name__ == '__main__':

    robot = hw_sdk_robot.Board()
    robot.enable_reception()
    test_board_rgb(robot)
    sonar = hw_sdk_sonar.Sonar()
    test_sonar(sonar)
    test_servos(robot)

    qs = Queue() 
    qm = Queue()

    thread_vision = threading.Thread(target=robot_see, args=(qs,qm,))
    thread_vision.start()

    thread_servo   = threading.Thread(target=robot_servo, args=(qs,robot))
    thread_mecanum = threading.Thread(target=robot_mecanum, args=(qm,))
    thread_servo.start()
    thread_mecanum.start()

    thread_vision.join()

    qs.join()
    qm.join()
    print(f"Queue Servo   : {qs.qsize()}")
    print(f"Queue Mecanum : {qm.qsize()}")
