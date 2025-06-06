import numpy
import yaml
import cv2

calbiration_filename = 'calibration.yaml' 
with open(calbiration_filename,'r') as file:
    color_ranges = yaml.safe_load(file)

#cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
cap = cv2.VideoCapture(0,cv2.CAP_MSMF)
assert cap.isOpened(), "Error opening camera."

#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

'''
    print("CV_CAP_PROP_FRAME_WIDTH: '{}'".format(cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
    print("CV_CAP_PROP_FRAME_HEIGHT : '{}'".format(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    print("CAP_PROP_FPS : '{}'".format(cap.get(cv2.CAP_PROP_FPS)))
    print("CAP_PROP_POS_MSEC : '{}'".format(cap.get(cv2.CAP_PROP_POS_MSEC)))
    print("CAP_PROP_FRAME_COUNT  : '{}'".format(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
    print("CAP_PROP_BRIGHTNESS : '{}'".format(cap.get(cv2.CAP_PROP_BRIGHTNESS)))
    print("CAP_PROP_CONTRAST : '{}'".format(cap.get(cv2.CAP_PROP_CONTRAST)))
    print("CAP_PROP_SATURATION : '{}'".format(cap.get(cv2.CAP_PROP_SATURATION)))
    print("CAP_PROP_HUE : '{}'".format(cap.get(cv2.CAP_PROP_HUE)))
    print("CAP_PROP_GAIN  : '{}'".format(cap.get(cv2.CAP_PROP_GAIN)))
    print("CAP_PROP_CONVERT_RGB : '{}'".format(cap.get(cv2.CAP_PROP_CONVERT_RGB)))
'''

while True:
    ret, frame = cap.read()
    assert ret, "Error receiving video frame"
    
    cv2.imshow('frame', frame)
    
    if cv2.waitKey(10) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)

for i in range(len(color_ranges)) :

    r = cv2.selectROI(frame)

    frame_crop = frame_hsv[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2]),:]

    average = [0,0,0]
    std     = [0,0,0]
    average[0] = numpy.average(frame_crop[:,:,0])
    std[0]     = numpy.std(frame_crop[:,:,0])
    average[1] = numpy.average(frame_crop[:,:,1])
    std[1]     = numpy.std(frame_crop[:,:,1])
    average[2] = numpy.average(frame_crop[:,:,2])
    std[2]     = numpy.std(frame_crop[:,:,2])

    found_color = False
    for color in color_ranges:
        ret = all(x<=y<=z for x,y,z in zip(color_ranges[color]['min'],average,color_ranges[color]['max']))
        if ret :  
            print("The color is: ",color)
            found_color = True
        else:
            match_color_low  = [x-y for x,y in zip(average, std)]
            match_color_high = [x+y for x,y in zip(average, std)]
            ret1 = all(x<=y<=z for x,y,z in zip(color_ranges[color]['min'],match_color_low,color_ranges[color]['max']))
            ret2 = all(x<=y<=z for x,y,z in zip(color_ranges[color]['min'],match_color_high,color_ranges[color]['max']))
            if ret1 or ret2 :  
                print("The color may be: ",color)
                found_color = True
    if not found_color : print("Unkown color")
    print("-----------------------------\n")