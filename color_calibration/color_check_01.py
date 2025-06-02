import numpy
import yaml
import cv2

calbiration_filename = 'calibration.yaml' 
with open(calbiration_filename,'r') as file:
        color_ranges = yaml.safe_load(file)

cap = cv2.VideoCapture(1,cv2.CAP_MSMF)
cap.set(cv2.CAP_PROP_FPS, 30.0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

assert cap.isOpened(), "Error opening camera."

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
        
        average = [0,0,0]
        std     = [0,0,0]
        average[0] = numpy.average(frame_hsv[:,:,0])
        std[0]     = numpy.std(frame_hsv[:,:,0])
        average[1] = numpy.average(frame_hsv[:,:,1])
        std[1]     = numpy.std(frame_hsv[:,:,1])
        average[2] = numpy.average(frame_hsv[:,:,2])
        std[2]     = numpy.std(frame_hsv[:,:,2])

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
                
             
cap.release()
cv2.destroyAllWindows()