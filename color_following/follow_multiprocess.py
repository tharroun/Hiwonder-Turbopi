#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/hiwonder/Github/Hiwonder-Turbopi')
sys.path.append('/home/hiwonder/Github/Hiwonder-Turbopi/sdk')
import time
import multiprocessing
from enum import Enum
import yaml
import numpy
import mypid
import hw_sdk_robot
import hw_sdk_sonar
import tah_sdk_mecanum
import cv2

class PROCESS_ACTION(Enum):
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

class CAMERA_ORIENTATION(Enum):
    LEFT    = 1
    CENTER  = 2
    RIGHT   = 3


# ==============================================================
def robot_servo(qs,Global,robot) -> int:

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
        if data is PROCESS_ACTION.KILL_THREAD:
            robot.pwm_servo_set_position(0.05, [[1, servo1_center]])
            robot.pwm_servo_set_position(0.05, [[2, servo2_center]])
            qs.task_done()
            break
        elif data is PROCESS_ACTION.LOST_OBJECT:
            pass
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
            robot.pwm_servo_set_position(0.01, [[1, new_y_position],[2, new_x_position]]) 
            servo_x_position = new_x_position
            servo_y_position = new_y_position
            if servo_x_position > 1800 :   Global.looking = CAMERA_ORIENTATION.LEFT
            elif servo_x_position < 1200 : Global.looking = CAMERA_ORIENTATION.RIGHT
            else :                         Global.looking = CAMERA_ORIENTATION.CENTER
        # ----------------------------------------------------------
        qs.task_done()
    
    #-------
    robot.pwm_servo_set_position(0.05, [[1, servo1_center]])
    robot.pwm_servo_set_position(0.05, [[2, servo2_center]])
    return 1 
# ==============================================================


# ==============================================================
def robot_mecanum(qm,Global,robot) -> int:

    mr = mypid.mypid(0.0035,0.0,0.0001)
    mz = mypid.mypid(1.5,0.0,0.0001)

    wheels = tah_sdk_mecanum.Mecanum(robot)

    (width,height) = qm.get()
    qm.task_done()
    assert (width,height)==(640,480) , print("First data in servo queue is not the width/height.")
    too_far_left  = 1 * width / 4
    too_far_right = 3 * width / 4

    while True:
        data = qm.get()
        # ----------------------------------------------------------
        # THE TASK
        if data is PROCESS_ACTION.KILL_THREAD:
            wheels.reset_motors()
            qm.task_done()
            break
        elif data is PROCESS_ACTION.LOST_OBJECT:
            wheels.reset_motors()
        else :
            # ----------------------------------------------------------
            controlr = 0
            if data[0] < too_far_left and Global.looking==CAMERA_ORIENTATION.LEFT :
                controlr = -numpy.fabs(mr.pid(too_far_left,data[0],data[3]))
                if controlr < -2.0 : controlr = -2.0
            if data[0] > too_far_right and Global.looking==CAMERA_ORIENTATION.RIGHT :
                controlr = numpy.fabs(mr.pid(too_far_right,data[0],data[3]))
                if controlr > 2.0 : controlr = 2.0
            # ----------------------------------------------------------
            controlz = 0
            direction = 0
            if numpy.fabs(data[2] - set_radius) > 10:
                controlz = int(mz.pid(set_radius,data[2],data[3]))
                if controlz < 0 :
                    if Global.looking==CAMERA_ORIENTATION.LEFT :
                        direction = 150
                    elif Global.looking==CAMERA_ORIENTATION.RIGHT :
                        direction = 210
                    else : direction = 180
                else : 
                    if Global.looking==CAMERA_ORIENTATION.LEFT :
                        direction = 30
                    elif Global.looking==CAMERA_ORIENTATION.RIGHT :
                        direction = 330
                    else : direction = 0
                controlz = numpy.abs(controlz)
                controlz = 40 if controlz > 40 else controlz
            # ----------------------------------------------------------
            wheels.set_velocity(controlz,direction,controlr)
        # ----------------------------------------------------------
        qm.task_done()
    
    #-------
    wheels.reset_motors()
    return 1 
# ==============================================================

# ==============================================================
def robot_see(qs,qm,cap) -> int:
    
    width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH )   
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT ) 
    qs.put((width,height))
    qm.put((width,height))

    t1 = time.perf_counter()
    while True:
        result, frame = cap.read()
        if not result:
            print("Error receiving video frame")
            qs.put(PROCESS_ACTION.KILL_THREAD)
            qm.put(PROCESS_ACTION.KILL_THREAD)
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
            # ------------------------------------------------
            #area = cv2.contourArea(contours[0])
            #arclength = cv2.arcLength(contours[0],True)
            #circularity = 4*numpy.pi*area/(arclength*arclength)
            # NEXT TIME:https://docs.opencv.org/4.11.0/d4/d70/tutorial_hough_circle.html
            # ------------------------------------------------
            M = cv2.moments(contours[0])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            radius = int(numpy.sqrt(M['m00']/numpy.pi))
            # ------------------------------------------------
            #if circularity < 0.7 : 
            if radius < 10 :
                qs.put(PROCESS_ACTION.LOST_OBJECT)
                qm.put(PROCESS_ACTION.LOST_OBJECT)
            else:
                qs.put((cx,cy,radius,t2-t1))
                qm.put((cx,cy,radius,t2-t1))
            #cv2.circle(frame,(cx,cy),radius,(0,255,0),5)
            cv2.drawContours(frame,contours,0,(0,0,255),5)
            t1 = t2
            # ------------------------------------------------
        else :
            qs.put(PROCESS_ACTION.LOST_OBJECT)
            qm.put(PROCESS_ACTION.LOST_OBJECT)
        cv2.imshow('frame', frame)

        if cv2.waitKey(10) == ord('q'):
            qs.put(PROCESS_ACTION.KILL_THREAD)
            qm.put(PROCESS_ACTION.KILL_THREAD)
            break
        # ------------------------------------------------
    return 1
# ==============================================================

# ==============================================================
def test_servos(robot) -> int: 
    print("TESTING SERVO 1 UP/DOWN...")
    robot.pwm_servo_set_position(0.3, [[1, servo2_center]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[1, servo2_center+400]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[1, servo2_center-400]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[1, servo2_center]]) 
    time.sleep(0.5)
    
    print("TESTING SERVO 2 LEFT/RIGHT...")
    robot.pwm_servo_set_position(0.3, [[2, servo2_center]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[2, servo2_center+500]]) 
    time.sleep(0.5)
    robot.pwm_servo_set_position(0.3, [[2, servo2_center-500]]) 
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
    sonar.setPixelColor(0, (100, 255, 100))
    sonar.setPixelColor(1, (100, 255, 100))
#    sonar.show()
    return 0
# ==============================================================

# ==============================================================
if __name__ == '__main__':

    Manager        = multiprocessing.Manager()
    Global         = Manager.Namespace()
    Global.looking = CAMERA_ORIENTATION.CENTER

    robot = hw_sdk_robot.Board()
    robot.enable_reception()
    test_board_rgb(robot)
    sonar = hw_sdk_sonar.Sonar()
    test_sonar(sonar)
    test_servos(robot)

    cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
    assert cap.isOpened(), print("Error opening camera")

    qs = multiprocessing.JoinableQueue() 
    qm = multiprocessing.JoinableQueue()

    thread_vision = multiprocessing.Process(target=robot_see, args=(qs,qm,cap))
    thread_vision.start()

    thread_servo   = multiprocessing.Process(target=robot_servo, args=(qs,Global,robot))
    thread_mecanum = multiprocessing.Process(target=robot_mecanum, args=(qm,Global,robot))
    thread_servo.start()
    thread_mecanum.start()

    thread_vision.join()
    cap.release()
    cv2.destroyAllWindows()
    
    qs.join()
    qm.join()

    thread_servo.terminate()
    thread_mecanum.terminate()
    thread_vision.terminate()

    print(f"Queue Servo   : {qs.qsize()}")
    print(f"Queue Mecanum : {qm.qsize()}")
