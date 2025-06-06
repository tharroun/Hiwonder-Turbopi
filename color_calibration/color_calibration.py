import numpy
import yaml
import cv2

color_list = ['pink','red','orange','yellow','green','blue','light_blue','purple','brown']
calbiration_filename = 'calibration.yaml' 
with open(calbiration_filename,'w') as file:
    file.close() 

#cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
cap = cv2.VideoCapture(0,cv2.CAP_MSMF)
assert cap.isOpened(), "Error opening camera."

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
        name = "color_standard.png"
        cv2.imwrite(name, frame)
        break
cap.release()
cv2.destroyAllWindows()

frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)

for color in color_list :

    print(f"Select color: {color}")
    r = cv2.selectROI(frame)

    frame_crop = frame_hsv[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2]),:]
    name = f"{color}.png"
    cv2.imwrite(name, frame_crop)

    '''
    nstd    = 2
    #HUE 
    average = numpy.rint(numpy.average(frame_crop[:,:,0]))
    std     = numpy.rint(numpy.std(frame_crop[:,:,0]))
    low_0  = average - nstd*std 
    high_0 = average + nstd*std
    #SAT 
    average = numpy.rint(numpy.average(frame_crop[:,:,1]))
    std     = numpy.rint(numpy.std(frame_crop[:,:,1]))
    low_1  = average - nstd*std 
    high_1 = average + nstd*std
    #VAL 
    average = numpy.rint(numpy.average(frame_crop[:,:,2]))
    std     = numpy.rint(numpy.std(frame_crop[:,:,2]))
    #low_2  = average - nstd*std 
    #high_2 = average + nstd*std
    low_2  = numpy.rint(numpy.min(frame_crop[:,:,2]))
    high_2 = numpy.rint(numpy.max(frame_crop[:,:,2]))

    low_0  = numpy.clip(low_0,  a_min=0, a_max=255)
    high_0 = numpy.clip(high_0, a_min=0, a_max=255)
    low_1  = numpy.clip(low_1,  a_min=0, a_max=255)  
    high_1 = numpy.clip(high_1, a_min=0, a_max=255)
    low_2  = numpy.clip(low_2,  a_min=0, a_max=255)  
    high_2 = numpy.clip(high_2, a_min=0, a_max=255)
    '''
    low_0  = numpy.min(frame_crop[:,:,0])
    high_0 = numpy.max(frame_crop[:,:,0])
    low_1  = numpy.min(frame_crop[:,:,1])
    high_1 = numpy.max(frame_crop[:,:,1])
    low_2  = numpy.min(frame_crop[:,:,2])
    high_2 = numpy.max(frame_crop[:,:,2])

    data = {color: {'min': [int(low_0),int(low_1),int(low_2)], 'max': [int(high_0),int(high_1),int(high_2)]}}
    with open(calbiration_filename,'a') as file:
        yaml.dump(data,file)