import cv2
import numpy as np

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml') # Must be included at the same directory

cap = cv2.VideoCapture('http://192.168.0.107:8080/video') #'0' is default port for built-in webcam camera, yours might be different

if (cv2.VideoCapture.isOpened(cap)):
    print("True")
if (cap.read()[0]):
    print("SecondTestPassed")
while True:
    ret, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5) # 2nd argument is compression factor; 3rd is to filter out false positives
    min_X = 999999 # Note: x is in pixels, set to very high number at first
    detected = False
    for (x,y,w,h) in faces:
        cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0), 2)
        min_X = min(min_X, x)  # Find the leftmost position of the bounding boxes
        detected = True

    if detected: print(min_X) # Print out the result on terminal, we will feed this thus Arduino for the motors
    window_name = 'MyWebcam'
    cv2.imshow(window_name,img)
    k = cv2.waitKey(30) & 0xff # How to quit the window, press 'Esc' key
    if k == 27:
        # filename = 'savedImage.jpg'
        # cv2.imwrite(filename, img)
        break

cap.release()
cv2.destroyAllWindows()