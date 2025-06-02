import numpy
import yaml
import cv2

color_list = ['pink','red','orange','yellow','green','blue','light_blue','purple','brown']
calbiration_filename = 'calibration.yaml' 
with open(calbiration_filename,'w') as file:
    file.close() 

for color in color_list :
    print(f"Place color: {color}")
    #cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
    cap = cv2.VideoCapture(1,cv2.CAP_MSMF)
    cap.set(cv2.CAP_PROP_FPS, 30.0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    assert cap.isOpened(), "Error opening camera."

    while True:
        ret, frame = cap.read()
        assert ret, "Error receiving video frame"
    
        cv2.imshow('frame', frame)
    
        if cv2.waitKey(10) == ord('q'):
            name = f"{color}.png"
            cv2.imwrite(name, frame)
            break
    cap.release()
    cv2.destroyAllWindows()

    frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    assert frame_hsv.shape == (480,640,3), "Unexpected frame_hsv shape"

    nstd    = 2
    #HUE 
    average = numpy.rint(numpy.average(frame_hsv[:,:,0]))
    std     = numpy.rint(numpy.std(frame_hsv[:,:,0]))
    low_h  = average - nstd*std 
    high_h = average + nstd*std
    #SAT 
    average = numpy.rint(numpy.average(frame_hsv[:,:,1]))
    std     = numpy.rint(numpy.std(frame_hsv[:,:,1]))
    low_s  = average - nstd*std 
    high_s = average + nstd*std
    #VAL 
    average = numpy.rint(numpy.average(frame_hsv[:,:,2]))
    std     = numpy.rint(numpy.std(frame_hsv[:,:,2]))
    low_v  = average - nstd*std 
    high_v = average + nstd*std

    low_h  = numpy.clip(low_h,  a_min=0, a_max=180)  
    high_h = numpy.clip(high_h, a_min=0, a_max=180)
    low_s  = numpy.clip(low_s,  a_min=0, a_max=255)  
    high_s = numpy.clip(high_s, a_min=0, a_max=255)
    low_v  = numpy.clip(low_v,  a_min=0, a_max=255)  
    high_v = numpy.clip(high_v, a_min=0, a_max=255)

    data = {color: {'min': [int(low_h),int(low_s),int(low_v)], 'max': [int(high_h),int(high_s),int(high_v)]}}
    with open(calbiration_filename,'a') as file:
        yaml.dump(data,file)
