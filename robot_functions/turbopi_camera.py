import numpy
import cv2

cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
#cap = cv2.VideoCapture(1,cv2.CAP_MSMF)
#cap.set(cv2.CAP_PROP_FPS, 30.0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Error opening camera")
    quit()

width  = cap.get(cv2.CAP_PROP_FRAME_WIDTH )   
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT ) 
print(f"wxh : {width} x {height}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error receiving video frame")
        quit()
    
    cv2.imshow('frame', frame)
    
    if cv2.waitKey(10) == ord('q'):
        cv2.imwrite("frame.png", frame)
        break
cap.release()
cv2.destroyAllWindows()
