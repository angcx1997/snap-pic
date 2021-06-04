import cv2, queue, threading, time
import numpy as np

face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml') # Must be included at the same directory

# bufferless VideoCapture
class VideoCapture:
  def __init__(self, name):
    self.cap = cv2.VideoCapture(name)
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      ret, frame = self.cap.read()
      if not ret:
        break
      if not self.q.empty():
        try:
          self.q.get_nowait()   # discard previous (unprocessed) frame
        except queue.Empty:
          pass
      self.q.put(frame)

  def read(self):
    return self.q.get()

cap = VideoCapture('http://192.168.0.107:8080/video')
while True:
    # time.sleep(.5)   # simulate time between events
    frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5) # 2nd argument is compression factor; 3rd is to filter out false positives
    detected = False

    min_X = 999999 # Note: x is in pixels, set to very high number at first
    for (x,y,w,h) in faces:
        cv2.rectangle(frame, (x,y), (x+w, y+h), (255,0,0), 2)
        min_X = min(min_X, x)  # Find the leftmost position of the bounding boxes
        detected = True

    if detected: print(min_X) 
    window_name = 'MyWebcam'
    cv2.imshow(window_name,frame)
    k = cv2.waitKey(30) & 0xff # How to quit the window, press 'Esc' key
    if k == 27:
        # filename = 'savedImage.jpg'
        # cv2.imwrite(filename, img)
        break

cap.release()
cv2.destroyAllWindows()
