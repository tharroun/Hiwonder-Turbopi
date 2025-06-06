#import sys
#sys.path.append('/home/hiwonder/')
import time
import threading
import math
from queue import Queue
from enum import Enum
from collections import deque
import numpy
import cv2
import yaml
#import hw_sdk_robot
#import hw_sdk_sonar
#import tah_sdk_mecanum
#import hw_sdk_sonar
#import hw_sdk_infrared

calbiration_filename = 'C:\\Users\\thadh\\Documents\\GitHub\\Hiwonder-Turbopi\\color_calibration\\calibration.yaml'
with open(calbiration_filename,'r') as file:
    color_ranges = yaml.safe_load(file)

COLOR_MIN = numpy.array(color_ranges['blue']['min'],numpy.uint8)
COLOR_MAX = numpy.array(color_ranges['blue']['max'],numpy.uint8)

small_kernel   = numpy.ones((3, 3), numpy.uint8)
medium_kernel  = numpy.ones((6, 6), numpy.uint8)
large_kernel   = numpy.ones((9, 9), numpy.uint8)

cap = cv2.VideoCapture(0,cv2.CAP_MSMF)
assert cap.isOpened(), "Error opening camera."
time.sleep(3)

while True:
    ret, frame = cap.read()
    assert ret, "Error receiving video frame"

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)

    color_mask = cv2.inRange(frame_hsv, COLOR_MIN, COLOR_MAX)
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, small_kernel,  iterations = 1)
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN,  medium_kernel, iterations = 1)
    cv2.imshow('frame', color_mask)

    if cv2.waitKey(10) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()