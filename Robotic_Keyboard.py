import numpy as np
import math
from dxl_control.Ax12 import Ax12
import time
import cv2, queue, threading
# import serial


# create motor object
my_dxl_1 = Ax12(4)
my_dxl_2 = Ax12(6)  #18A
my_dxl_3 = Ax12(1)  #3rd joint
my_dxl_4 = Ax12(2)  #4th
my_dxl_5 = Ax12(3)
dxl = list()
dxl.append(my_dxl_1)
dxl.append(my_dxl_2)
dxl.append(my_dxl_3)
dxl.append(my_dxl_4)
dxl.append(my_dxl_5)
# my_dxl_5 = Ax12(1)

# Connection
Ax12.open_port()
Ax12.set_baudrate()

# arduino = serial.Serial('COM5', 115200, timeout=.1)
# time.sleep(1) #give the connection a second to settle



# #Define macro
# #Link Length(in mm)
L1 = 17.03
Z1 = 36.21  # Need to add base height
L2 = 178
L3 = 108
X4 = 15
Y4 = 50
L5 = 70
goal_pos = np.zeros(3)
x=0
y=0
z=0
X_res = 320
offset3 = 20
offset4 = 5


joint_constraint = {    0: [0,300],
                        1: [95,229],
                        2: [0,287],
                        3: [80,248],
                        4: [80,260]}




theta = np.zeros(5)

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

def fk(theta):
    '''
    Forward kinematic up to 3rd link
    '''
    theta1 = math.radians(theta[0])
    theta2 = math.radians(theta[1])
    theta3 = math.radians(theta[2])

    T01 = np.array([[np.cos(theta1), 0, np.sin(theta1), L1 * np.cos(theta1)],
                    [np.sin(theta1), 0, -np.cos(theta1), L1 * np.sin(theta1)],
                    [0, 1, 0, Z1],
                    [0, 0, 0, 1]])

    T12 = np.array([[np.cos(theta2), -np.sin(theta2), 0, L2 * np.cos(theta2)],
                    [np.sin(theta2), np.cos(theta2), 0, L2 * np.sin(theta2)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    T23 = np.array([[np.cos(theta3), -np.sin(theta3), 0, L3 * np.cos(theta3)],
                    [np.sin(theta3), np.cos(theta3), 0, L3 * np.sin(theta3)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    T_mat = {'T12': T12,
             'T23': T23,}

    # Forward Kinematics
    T02 = T01 @ T12
    T03 = T02 @ T23

    T_mat = {'T01': T01,
             'T02': T02,
             'T03': T03,
             }

    pos = np.zeros(6)
    pos[0:3] = T03[0:3, 3]
    pos[3] = math.atan(T03[1, 0] / T03[0, 0])  # roll
    pos[4] = math.atan(-T03[2, 0] / np.linalg.norm([T03[2, 1], T03[2, 2]]))  # pitch
    if T03[2, 2] == 0:
        pos[5] = np.pi / 2
    else:
        pos[5] = math.atan(T03[2, 1] / T03[2, 2])

    return T_mat, pos[0:3]

def ik(goal_pos):
    '''
    Inverse Kinematic with trigonometry analysis up to third link
    '''
    X = goal_pos [0]
    Y = goal_pos [1]
    Z = goal_pos [2]

    r1 = (math.sqrt(pow(X, 2) + pow(Y, 2)))-20
    r2 = Z - 30
    r3 = math.sqrt(pow(r1, 2) + pow(r2, 2))

    theta1 = np.arctan(Y/X)
    a =(pow(95,2)-pow(140,2)-pow(r3,2))/(-2*140*r3)
    b = (-pow(95,2)-pow(140,2)+pow(r3,2))/(-2*140*95)

    phi1 = -np.arccos(a)
    phi2 = np.arctan(r2/r1)
    phi3 = -np.arccos(b)

    theta2 = phi2-phi1
    theta3 = np.pi - phi3
    


    theta[0:3] = np.degrees([theta1,theta2,theta3])

    return theta

def setMotorSpeed(speed):
    my_dxl_1.set_moving_speed(speed)
    my_dxl_2.set_moving_speed(speed)
    my_dxl_3.set_moving_speed(speed)
    my_dxl_4.set_moving_speed(speed)
    my_dxl_5.set_moving_speed(speed)

def getPosition():
    my_dxl_1.get_position()
    my_dxl_2.get_position()
    my_dxl_3.get_position()
    my_dxl_4.get_position()
    my_dxl_5.get_position()

def setJointPosition(theta):
    '''
    1. Convert theta calculated to fit dynamixel range
    2. Set respective joint angle
    '''
    #Convert range[-150,150] to [0,300] to fulfill motor needs
    theta300 = theta.copy()
    for i,_ in enumerate(theta300):
        if i == 2 or i ==4 or i == 0:
            if theta300[i] < 180:
                tmp = abs(theta300[i]+150)  #Convert from range of (-150,150) to (0,300)
            elif theta[i] >= 180:
                tmp = - (360 - abs(theta300[i]+150)) 
        elif i == 3 : 
            if -theta300[i] < 180:
                tmp = abs(-theta300[i]+150)  #Convert from range of (-150,150) to (0,300)
            elif -theta300[i] >= 180:
                tmp = - (360 - abs(-theta300[i]+150))
        elif i == 1:
            tmp = abs(theta300[i]+95)  
        
        theta300[i] = tmp
        [min,max] = joint_constraint[i]
        if (tmp > max or tmp < min) or np.isnan(tmp):
            print("out of bound",i)
            return False

    #Set joint angle to changed angle
    for i,j in enumerate(theta300):
        motorID = dxl[i]
        motorID.set_position(j)
    return True

def test_ismoving():
    for i in dxl:
        if i.is_moving() == 1:
            return True
    return False


def checkCoor(goal_pos):
    x_constraint = [100,150]
    y_constraint = [100,150]
    z_constraint = [200,240]

    if (goal_pos[0] > x_constraint[0] or goal_pos[1] > y_constraint[0]) and (goal_pos[2] > z_constraint[0]):
        print('coor constraint on wedges')
        return False
    
    elif (goal_pos[0] > x_constraint[1]) or (goal_pos[1] > y_constraint[1]) or (goal_pos[2] > z_constraint[1]):
        print('coor constraint or max ')
        return False
    else: 
        return True

def main():
    #Set initial Position
    # face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml') # Must be included at the same directory
    # cap = VideoCapture('http://10.27.14.50:8080/video') #'0' is default port for built-in webcam camera, yours might be different


    coor_input = ['w','s','a','d','p','o']
    orien_input = ['8','5','4','6']
    theta1 = 0
    theta2 = 135
    theta3 = -60
    theta4 = -25
    theta5 = 0
    theta = [theta1, theta2, theta3, theta4, theta5]
    _, pos = fk(theta)
    goal_pos = pos[0:3]
    print("\nInitial position: ",goal_pos)
    print("Initial angle:",theta)
    setMotorSpeed(40)
    setJointPosition(theta)
    


    cameraOn = True
    # calibration = True
    #Step 2: Camera positioning
    while cameraOn:
        # img = cap.read()
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # # Turn camera on and feed video stream

        # while calibration:
        #     img = cap.read()
        #     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #     faces = face_cascade.detectMultiScale(gray, 1.3, 5) # 2nd argument is compression factor; 3rd is to filter out false positives
        #     detected = False
        #     min_X = 9999999
        #     max_X = 0
        #     for (x,y,w,h) in faces:
        #         cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0), 2)
        #         min_X = min(min_X, x)  # Find the leftmost position of the bounding boxes
        #         max_X = max(max_X,x+h)
        #         detected = True
                
        #     if detected:
        #         avg_X = np.average([min_X,max_X])
        #         print(avg_X)
        #         if avg_X > (X_res//2 + 10):
        #             theta[0] -= 1
        #         elif avg_X < (X_res // 2 - 10):
        #             theta[0] += 1
        #         else:
        #             calibration = False
        #             break
        #         setJointPosition(theta)
        #         # time.sleep(1)

        # window_name = 'MyWebcam'
        # cv2.imshow(window_name,img)
        # k = cv2.waitKey(30) & 0xff # How to quit the window, press 'Esc' key
        # if k == 27:
        #     # filename = 'savedImage.jpg'
        #     # cv2.imwrite(filename, img)
        #     break

        user_input = True

        #Step 3 : IK / User Input
        while user_input:
            tmp_pos = goal_pos.copy()
            tmp_theta = theta.copy()

            # key = arduino.readline()[0:-2].decode('utf-8')
            key = input("\nDirection key : ")
            # print(key)
            #Adjusting coordinate 
            if key in coor_input:
                print(key)
                print("Key in coor_input")
                if key == 'w': #up
                    goal_pos[2] += 10
                elif key == 's': #down
                    goal_pos[2] -= 10
                elif key == 'a': #left
                    goal_pos[1] -= 10
                elif key == 'd': #right
                    goal_pos[1] += 10
                elif key == 'o': #left`
                    goal_pos[0] -= 10
                elif key == 'p': #right
                    goal_pos[0] += 10
                theta = ik(goal_pos)
                theta[3] = (-(theta[1]+theta[2])+offset3)
                theta[4] = (-theta[0]+offset4)


            #Adjusting orientation
            elif key in orien_input:
                print(key)
                print("Key in orient input")
                if key == '8': #tilt up
                    theta[3] += 5
                elif key == '5': #tilt down
                    theta[3] -= 5
                elif key == '4': #tilt left
                    theta[4] -= 5
                elif key == '6': #tilt right
                    theta[4] += 5

            elif key == 'q':  # exit key
                user_input = False
                cameraOn = False
                break

            

            print("Check angles: ", theta)
            print("Goal position: ", goal_pos)
            angle_cond = setJointPosition(theta)
            # if checkCoor(goal_pos):
            #     angle_cond = setJointPosition(theta)
            # else:
            #     angle_cond = False

            # if (angle_cond == False) or (checkCoor(goal_pos) == False):
            if angle_cond == False:
                goal_pos = tmp_pos
                theta = tmp_theta
                print('\nNo solution')
                print("Previous Pos:", goal_pos)
                continue 

            _, pos = fk(theta)
            print("Check position(with FK): ", pos)

            # while not test_ismoving():
            #     print('is moving')

        
if __name__ == '__main__':
    main()
    my_dxl_1.disable_torque()
    my_dxl_2.disable_torque()
    my_dxl_3.disable_torque()
    my_dxl_4.disable_torque()
    my_dxl_5.disable_torque()
    Ax12.close_port()
