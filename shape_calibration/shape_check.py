import sys
import numpy
import yaml
import cv2

#----------------------
calbiration_filename = 'C:\\Users\\thadh\\Documents\\GitHub\\Hiwonder-Turbopi\\color_calibration\\calibration.yaml' 
with open(calbiration_filename,'r') as file:
        color_ranges = yaml.safe_load(file)

color_min = numpy.array(color_ranges['blue']['min'],numpy.uint8)
color_max = numpy.array(color_ranges['blue']['max'],numpy.uint8)
print(color_min,color_max)
#--------------------

#----------------------
calbiration_filename = 'C:\\Users\\thadh\\Documents\\GitHub\\Hiwonder-Turbopi\\shape_calibration\\calibration.yaml' 
with open(calbiration_filename,'r') as file:
        shape_calibration_data = yaml.safe_load(file)

shape_names = list(shape_calibration_data.keys())
shape_data = numpy.zeros((len(shape_names),5))
for i,shape in enumerate(shape_names) :
        shape_data[i][0] = shape_calibration_data[shape]['HM_AVG'][0]
        shape_data[i][1] = shape_calibration_data[shape]['HM_AVG'][1]
        shape_data[i][2] = shape_calibration_data[shape]['HM_AVG'][2]
        shape_data[i][3] = shape_calibration_data[shape]['HM_AVG'][3]
        shape_data[i][4] = shape_calibration_data[shape]['AREA_AVG'][0]
#----------------------

cap = cv2.VideoCapture(1,cv2.CAP_MSMF)
cap.set(cv2.CAP_PROP_FPS, 30.0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    assert ret, "Error receiving video frame"
    
    cv2.imshow('frame', frame)
    key = cv2.waitKey(10)

    if  key == ord('q'):
        break
    elif key == ord('c') :
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        assert frame_hsv.shape == (480,640,3), "Unexpected frame_hsv shape"
        color_mask = cv2.inRange(frame_hsv, color_min, color_max)
        cv2.imwrite("test.png",color_mask)

cap.release()
cv2.destroyAllWindows