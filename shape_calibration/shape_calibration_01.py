import numpy
import yaml
import cv2

small_kernel   = numpy.ones((3, 3), numpy.uint8)
medium_kernel  = numpy.ones((6, 6), numpy.uint8)
large_kernel   = numpy.ones((9, 9), numpy.uint8)

shape_list = ['heart']
calbiration_filename = 'calibration.yaml' 
with open(calbiration_filename,'w') as file:
    file.close() 

for shape in shape_list :
    print(f"Place shape: {shape}")
    #cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
    cap = cv2.VideoCapture(1,cv2.CAP_MSMF)
    cap.set(cv2.CAP_PROP_FPS, 30.0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    assert cap.isOpened(), "Error opening camera."

    while True:
        ret, frame = cap.read()
        assert ret, "Error receiving video frame"
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret,frame_bw = cv2.threshold(frame, 55, 255, cv2.THRESH_BINARY_INV)
        cv2.imshow('frame_bw', frame_bw)
    
        if cv2.waitKey(10) == ord('k'):
            break
    cap.release()
    cv2.destroyAllWindows()

    assert frame_bw.shape == (480,640), "Unexpected frame_hsv shape"
    frame_bw = cv2.morphologyEx(frame_bw, cv2.MORPH_CLOSE, small_kernel, iterations = 1)
    frame_bw = cv2.morphologyEx(frame_bw, cv2.MORPH_OPEN,  small_kernel, iterations = 1)

    contours,hierarchy = cv2.findContours(frame_bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours)} contours")
    contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

    shapeMoments = numpy.array([])
    shapeAreas   = numpy.array([])
    for contour in contours :
        x,y,w,h = cv2.boundingRect(contour)
        #frame_roi = numpy.ones((h,w,3),dtype=numpy.uint8)*255
        #frame_roi[:,:,0] = frame[y:y+h,x:x+w]
        #frame_roi[:,:,1] = frame[y:y+h,x:x+w]
        #frame_roi[:,:,2] = frame[y:y+h,x:x+w]
        frame_roi = numpy.dstack([frame[y:y+h,x:x+w],frame[y:y+h,x:x+w],frame[y:y+h,x:x+w]])/255
        contour_roi = contour - [x,y]
        frame_roi = cv2.drawContours(frame_roi, [contour_roi], -1, (255,0,0), 3)
        cv2.imshow("name", frame_roi)

        if cv2.waitKey(0) == ord('k'):
            #-----------------
            huMoments = cv2.HuMoments(cv2.moments(frame_roi[:,:,0]) )
            huMoments = -numpy.sign(huMoments) * numpy.log10(numpy.fabs(huMoments)) 
            shapeMoments = numpy.append(shapeMoments,huMoments)
            #-----------------
            contour_area = cv2.contourArea(contour)
            hull_area    = cv2.contourArea(cv2.convexHull(contour))
            shapeAreas = numpy.append(shapeAreas,contour_area/hull_area)
            #-----------------
            frame = cv2.drawContours(frame, [contour], -1, (255,0,0), 3)
        # ================
        cv2.destroyAllWindows()
    name = f"{shape}.png"
    cv2.imwrite(name, frame)
    shapeMoments = numpy.reshape(shapeMoments,(-1,7))
    sM_avg = numpy.average(shapeMoments,axis=0)
    sM_std = numpy.std(shapeMoments,axis=0)
    sA_avg = numpy.average(shapeAreas)
    sA_std = numpy.std(shapeAreas)
    data = {shape: \
              {'HM_AVG': [float(sM_avg[0]),float(sM_avg[1]),float(sM_avg[2]),float(sM_avg[3])], \
               'HM_STD': [float(sM_std[0]),float(sM_std[1]),float(sM_std[2]),float(sM_std[3])], \
               'AREA_AVG': [float(sA_avg)],
               'AREA_STD': [float(sA_std)]}}
    with open(calbiration_filename,'a') as file:
        yaml.dump(data,file)
