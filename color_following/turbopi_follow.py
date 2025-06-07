#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/hiwonder/Github/Hiwonder-Turbopi')
import time
import threading
import math
from queue import Queue
from enum import Enum
from collections import deque
import numpy
import cv2
import hw_sdk_robot
import hw_sdk_sonar
import tah_sdk_mecanum
#import hw_sdk_sonar
#import hw_sdk_infrared

#GREEN_MIN = numpy.array([60, 132, 32],numpy.uint8)
#GREEN_MAX = numpy.array([84, 236, 96],numpy.uint8)
GREEN_MIN = numpy.array([36, 100, 10],numpy.uint8) # OPEN WORLD
GREEN_MAX = numpy.array([86, 255, 255],numpy.uint8)
PURPLE_MIN = numpy.array([150, 132, 90],numpy.uint8)
PURPLE_MAX = numpy.array([162, 184, 130],numpy.uint8)
COLOR_MIN = GREEN_MIN
COLOR_MAX = GREEN_MAX

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
set_radius    = 25

# ==============================================================
def robot_move(in_q,robot) -> Enum:
    kpid_x = (0.25,0.005,0.05)
    kpid_y = (0.2,0.002,0.02)
    kpid_R = (0.005,0.0001,0.00005)
    integral_x = 0.0
    integral_y = 0.0
    integral_R = 0.0
    previous_error_x = 0.0
    previous_error_y = 0.0
    previous_error_R = 0.0
    servo_x = servo2_center
    servo_y = servo1_center

    width,height = in_q.get()
    in_q.task_done()
    print("Width: ", width, "Height: ", height)
    cX = width/2.0
    cY = height/2.0
    
    wheels = tah_sdk_mecanum.Mecanum(robot)
    
    dq = deque(maxlen=5)
    # --------------------------------------------------------
    while True:
        data = in_q.get()
        dq.append(data)
        if dq[-1] is _sentinel:
            in_q.put(_sentinel)
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
                error = cX - dq[-1][0]
                integral_x += error*dq[-1][3]
                derivative = (error - previous_error_x)/dq[-1][3]
                control = kpid_x[0] * error + kpid_x[1] * integral_x + kpid_x[2] * derivative
                previous_error_x = error
                position = int(servo_x + control)
                if position < 1000 :
                    wheels.set_velocity(0,90,0.2)
                    time.sleep(0.1)
                    wheels.reset_motors()
                    position = servo_x     
                elif position > 2000:
                    wheels.set_velocity(0,90,-0.2)
                    time.sleep(0.1)
                    wheels.reset_motors()
                    position = servo_x 
                robot.pwm_servo_set_position(0.3, [[2, position]]) 
                servo_x = position
                # ----------------------------------------------------------
                error = cY - dq[-1][1]
                integral_y += error*dq[-1][3]
                derivative = (error - previous_error_y)/dq[-1][3]
                control = kpid_y[0] * error + kpid_y[1] * integral_y + kpid_y[2] * derivative
                previous_error_y = error
                position = int(servo_y - control)
                if position < 1000 or position > 2000:
                    position = servo_y
                robot.pwm_servo_set_position(0.3, [[1, position]]) 
                servo_y = position
            # ----------------------------------------------------------
            if MOVE:
                error = set_radius - dq[-1][2]
                integral_R += error*dq[-1][3]
                derivative = (error - previous_error_R)/dq[-1][3]
                control = kpid_R[0] * error + kpid_R[1] * integral_R + kpid_R[2] * derivative
                previous_error_x = error
                if control < 0:
                    wheels.set_velocity(30,180,0)
                    time.sleep(math.fabs(control)/2)
                    wheels.reset_motors()
                else :
                    wheels.set_velocity(30,0,0)
                    time.sleep(math.fabs(control))
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
        
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        color_mask = cv2.inRange(frame_hsv, COLOR_MIN, COLOR_MAX)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, small_kernel, iterations = 1)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, medium_kernel,iterations = 1)
        #cv2.imshow('color_mask', color_mask)

        """
        # OLD METHOD : DOES NOT CALCULATE THE RADIUS OF THE BLOB TO GAUGE THE DISTANCE.
        keypoints = blob_detector.detect(color_mask)
        if (keypoints) : 
            t2 = time.perf_counter()
            out_q.put((keypoints[0].pt[0],keypoints[0].pt[1],t2-t1))
            frame = cv2.drawKeypoints(frame, keypoints, numpy.array([]), (255,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            t1 = t2
        cv2.imshow('frame', frame)
        """
        
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
    thread_motion = threading.Thread(target=robot_move, args=(q,robot))
    thread_vision = threading.Thread(target=robot_see, args=(q,))
    
    thread_motion.start()
    thread_vision.start()
    q.join()

