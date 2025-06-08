#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/hiwonder/Github/Hiwonder-Turbopi')
sys.path.append('/home/hiwonder/Github/Hiwonder-Turbopi/sdk')
import time
import threading
import math
import yaml
from queue import Queue
from enum import Enum
from collections import deque
import numpy
import cv2
import hw_sdk_robot
import hw_sdk_sonar
import tah_sdk_mecanum
import mypid
#import hw_sdk_sonar
#import hw_sdk_infrared


# -------------
# LOAD THE COLOR DEFINITIONS
calbiration_filename = '/home/hiwonder/Github/Hiwonder-Turbopi/color_calibration/calibration.yaml' 
with open(calbiration_filename,'r') as file:
    color_ranges = yaml.safe_load(file)
COLOR_MIN = numpy.array(color_ranges['green']['min'],numpy.uint8)
COLOR_MAX = numpy.array(color_ranges['green']['max'],numpy.uint8)

"""
# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()
params.filterByColor = False
params.filterByArea = True
params.minArea = 100
params.filterByCircularity = False
params.filterByConvexity = False
params.filterByInertia = True
params.minInertiaRatio = 0.5
blob_detector = cv2.SimpleBlobDetector_create(params)
"""
small_kernel   = numpy.ones((3, 3), numpy.uint8)
medium_kernel  = numpy.ones((6, 6), numpy.uint8)
large_kernel   = numpy.ones((9, 9), numpy.uint8)

class Exitcode(Enum):
    GOOD       = 1
    ERROR_MOVE = 2
    ERROR_SEE  = 3

_sentinel = object() 

servo2_center = 1500
servo1_center = 1500
set_radius    = 40

# ==============================================================
def robot_move_1(in_q,robot) -> Enum:
    sx = mypid.mypid(0.15,0.0,0.005)
    sy = mypid.mypid(0.2,0.0,0.005)
    mr = mypid.mypid(0.005,0.0001,0.00005)

    servo_x = servo2_center
    servo_y = servo1_center

    width,height = in_q.get()
    in_q.task_done()
    assert width==640 , print("First data in queue is not the width/height.")
    cX = width/2.0
    cY = height/2.0
    
    wheels = tah_sdk_mecanum.Mecanum(robot)
    
    dq = deque(maxlen=5)
    # --------------------------------------------------------
    while True:
        data = in_q.get()
        dq.append(data)
        if dq[-1] is _sentinel:
            in_q.task_done()
            wheels.reset_motors()
            robot.pwm_servo_set_position(0.3, [[1, servo1_center]]) 
            robot.pwm_servo_set_position(0.3, [[2, servo2_center]])      
            break
        if len(dq)==5:
            dX = (dq[-1][0] - dq[0][0])/width
            dY = (dq[-1][1] - dq[0][1])/height
            dR = (dq[-1][2] - dq[0][2])
            
            if (math.fabs(dX) > 0.05) or (math.fabs(dY) > 0.05):
                SWIVEL = True
                MOVE = False
            else:
                SWIVEL = False
                MOVE = True
            # ----------------------------------------------------------
            if SWIVEL:
                control = sx.pid(cX, dq[-1][0], dq[-1][3])
                position_x = int(servo_x + control)
                if position_x < 1000 :
                    wheels.set_velocity(0,90,0.2)
                    time.sleep(0.1)
                    wheels.reset_motors()
                    position_x = int(servo_x - control/4.0)   
                elif position_x > 2000:
                    wheels.set_velocity(0,90,-0.2)
                    time.sleep(0.1)
                    wheels.reset_motors()
                    position_x = int(servo_x - control/4.0)
                servo_x = position_x
                # ----------------------------------------------------------
                control = sx.pid(cY, dq[-1][1], dq[-1][3])
                position_y = int(servo_y - control)
                if position_y < 1100 or position_y > 1900 : position_y = servo_y
                servo_y = position_y
                robot.pwm_servo_set_position(0.02, [[1, position_y],[2, position_x]]) 
            # ----------------------------------------------------------
            if MOVE:
                wheels.reset_motors()
            # ----------------------------------------------------------
        in_q.task_done()
    # --------------------------------------------------------
    return Exitcode.GOOD
# ==============================================================

# ==============================================================
def robot_move_0(in_q,robot) -> Enum:
    sx = mypid.mypid(0.15,0.0,0.005)
    sy = mypid.mypid(0.2,0.0,0.005)
    mz = mypid.mypid(0.005,0.0001,0.00005)

    servo_x = servo2_center
    servo_y = servo1_center

    width,height = in_q.get()
    in_q.task_done()
    assert width==640 , print("First data in queue is not the width/height.")
    cX = width/2.0
    cY = height/2.0
    
    wheels = tah_sdk_mecanum.Mecanum(robot)
    
    # --------------------------------------------------------
    while True:
        data = in_q.get()
        if data is _sentinel:
            in_q.task_done()
            wheels.reset_motors()
            robot.pwm_servo_set_position(0.3, [[1, servo1_center]]) 
            robot.pwm_servo_set_position(0.3, [[2, servo2_center]])      
            break
        # ----------------------------------------------------------
        else :
            control = sx.pid(cX, data[0], data[3])
            position_x = int(servo_x + control)
            if position_x < 1000 :
                wheels.set_velocity(0,90,0.2)
                time.sleep(0.1)
                wheels.reset_motors()
                position_x = int(servo_x - control/4.0)   
            elif position_x > 2000:
                wheels.set_velocity(0,90,-0.2)
                time.sleep(0.1)
                wheels.reset_motors()
                position_x = int(servo_x - control/4.0)
            # ----------------------------------------------------------
            control = sx.pid(cY, data[1], data[3])
            position_y = int(servo_y - control)
            if position_y < 1100 or position_y > 1900 : position_y = servo_y
            # ----------------------------------------------------------
            robot.pwm_servo_set_position(0.02, [[1, position_y],[2, position_x]]) 
            servo_x = position_x
            servo_y = position_y
            # ----------------------------------------------------------
            if numpy.fabs(data[2] - set_radius) > 10:
                control = mz.pid(set_radius,data[2],data[3])
                if control< 0 :
                    wheels.set_velocity(20,180,0.0)
                    
                else:
                    wheels.set_velocity(20,0,0.0)
                time.sleep(numpy.fabs(control))
            wheels.reset_motors()
            # ----------------------------------------------------------
        in_q.task_done()
    # --------------------------------------------------------
    return Exitcode.GOOD


# ==============================================================
def robot_see(out_q) -> Enum:

    cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
    #cap = cv2.VideoCapture(1,cv2.CAP_MSMF)
    #cap.set(cv2.CAP_PROP_FPS, 30.0)
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        print("Error opening camera")
        out_q.put(_sentinel)
        return Exitcode.ERROR_SEE

    width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH )   
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT ) 
    out_q.put((width,height))

    t1 = time.perf_counter()
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error receiving video frame")
            out_q.put(_sentinel)
            return Exitcode.ERROR_SEE
        
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
        color_mask = cv2.inRange(frame_hsv, COLOR_MIN, COLOR_MAX)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, small_kernel, iterations = 1)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, small_kernel,iterations = 1)
        #cv2.imshow('color_mask', color_mask)

        '''
        # OLD METHOD : DOES NOT CALCULATE THE RADIUS OF THE BLOB TO GAUGE THE DISTANCE.
        keypoints = blob_detector.detect(color_mask)
        if (keypoints) : 
            t2 = time.perf_counter()
            out_q.put((keypoints[0].pt[0],keypoints[0].pt[1],t2-t1))
            frame = cv2.drawKeypoints(frame, keypoints, numpy.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            t1 = t2
        cv2.imshow('frame', frame)
        '''
        
        contours,hierarchy = cv2.findContours(color_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if (contours):
            t2 = time.perf_counter()
            if (len(contours)>1) : 
                contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
            M = cv2.moments(contours[0])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            radius = int(numpy.sqrt(M['m00']/numpy.pi))
            out_q.put((cx,cy,radius,t2-t1))
            cv2.circle(frame,(cx,cy),radius,(0,255,0),5)
            t1 = t2
        cv2.imshow('frame', frame)

        if cv2.waitKey(10) == ord('q'):
            break
 
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    print("Queue", out_q.qsize())
    out_q.put(_sentinel) 
    return Exitcode.GOOD;

# ================================================
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
# ================================================

# ================================================
def test_board_rgb(robot) -> int: 
    print("TESTING BOARD RGB...")
    robot.set_rgb([[1, 0, 0, 0]])
    time.sleep(0.5)
    return 0
# ================================================

# ================================================
def test_sonar(sonar) -> int: 
    print("TESTING SONAR...")
    sonar.setRGBMode(0)
    time.sleep(0.1)
    sonar.setPixelColor(0, (255, 255, 255))
    sonar.setPixelColor(1, (255, 255, 255))
#    sonar.show()
    return 0
# ================================================

if __name__ == '__main__':
    
    robot = hw_sdk_robot.Board()
    robot.enable_reception()
    test_board_rgb(robot)
    sonar = hw_sdk_sonar.Sonar()
    test_sonar(sonar)
    test_servos(robot)

    q = Queue() 
    thread_motion = threading.Thread(target=robot_move_0, args=(q,robot))
    thread_vision = threading.Thread(target=robot_see, args=(q,))
    
    thread_motion.start()
    thread_vision.start()
    q.join()

