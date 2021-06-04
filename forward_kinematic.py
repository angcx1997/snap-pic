import matplotlib.pyplot as plt
import numpy as np
from math import radians
from dxl_control.Ax12 import Ax12
import time

# create motor object
my_dxl_2 = Ax12(6)  #18A
my_dxl_3 = Ax12(3)  #3rd joint
my_dxl_4 = Ax12(2)  #4th
my_dxl_5 = Ax12(4)
dxl = list()
dxl.append(my_dxl_2)
dxl.append(my_dxl_3)
dxl.append(my_dxl_4)
dxl.append(my_dxl_5)
# my_dxl_5 = Ax12(1)


# #Define macro
# #Link Length(in mm)
L1 = 20
Z1 = 30  #Need to add base height
L2 = 140
L3 = 95
X4 = 15
Y4 = 50
L5 = 70

# #Min and Max of joint angle
# J0_MIN =
# J0_MAX =
# J1_MIN =
# J1_MAX =
# J2_MIN =
# J2_MAX =
# J3_MIN =
# J3_MAX =


theta1 = 0
theta2 = 0
theta3 = 0
theta4 = 0
theta5 = 0
theta = [theta2,theta3,theta4,theta5]

translational_v = list()
rotational_v = list()


# connecting
Ax12.open_port()
Ax12.set_baudrate()


def fk():
    
    theta2 = theta[0]
    theta3 = theta[1]
    theta4 = theta[2]
    theta5 = theta[3]

    T01 = np.array([[np.cos(theta1),0,np.sin(theta1),L1*np.cos(theta1)],
                    [np.sin(theta1),0,-np.cos(theta1),L1*np.sin(theta1)],
                    [0,1,0,Z1],
                    [0,0,0,1]])

    T12 = np.array([[np.cos(theta2),-np.sin(theta2),0,L2*np.cos(theta2)],
                    [np.sin(theta2),np.cos(theta2),0,L2*np.sin(theta2)],
                    [0,0,1,0],
                    [0,0,0,1]])


    T23 = np.array([[np.cos(theta3),-np.sin(theta3),0,L3*np.cos(theta3)],
                    [np.sin(theta3),np.cos(theta3),0,L3*np.sin(theta3)],
                    [0,0,1,0],
                    [0,0,0,1]])

    T34 = np.array([[np.cos(theta4),0,-np.sin(theta4),X4*np.cos(theta4)-Y4*np.sin(theta4)],
                    [np.sin(theta4),0,np.cos(theta4),X4*np.sin(theta4)+Y4*np.cos(theta4)],
                    [0,-1,0,0],
                    [0,0,0,1]])

    T45 = np.array([[np.cos(theta5),-np.sin(theta5),0,L5*np.cos(theta5)],
                    [np.sin(theta5),np.cos(theta5),0,L5*np.sin(theta5)],
                    [0,1,0,0],
                    [0,0,0,1]])
    
    # T_mat = { 'T12': T12,
    #           'T23': T23,
    #           'T34': T34,
    #           'T45': T45,  }
    #Forward Kinematics
    T01 = T01
    T02 = T01@T12
    T03 = T02@T23
    T04 = T03@T34
    T05 = T04@T45

    # print(T_mat)
    print(T05)

def deg2rad(angle):
    rad = angle * np.pi/180
    return rad

def setMotorSpeed(speed):
    my_dxl_2.set_moving_speed(speed)
    my_dxl_3.set_moving_speed(speed)
    my_dxl_4.set_moving_speed(speed)
    my_dxl_5.set_moving_speed(speed)

def getPosition():
    my_dxl_2.get_position()
    my_dxl_3.get_position()
    my_dxl_4.get_position()
    my_dxl_5.get_position()

def user_input():
    """ Check to see if user wants to continue """
    ans = input('Continue? : y/n ')
    if ans == 'n':
        return False
    else:
        return True

def test_pos(motor_object):
    # desired angle input
    input_pos = int(input("input pos: "))

    return input_pos



def main():
    loop = True
    motor = {1: 'my_dxl_2', 2: 'my_dxl_3',3: 'my_dxl_4', 4: 'my_dxl_5'}
    setMotorSpeed(50)

    while loop:
        getPosition()

        inputPos = True
        while inputPos:
            for i,_ in enumerate(theta):
                motorID = dxl[i]
                theta[i] = int(input("input pos: "))
                if i == 1 or i ==3:
                    tmp = abs(theta[i]+150)  #Convert from range of (-150,150) to (0,300)
                

                elif i == 2 :
                    tmp = abs(-theta[i]+150) 

                elif i == 0:
                    tmp = abs(theta[i]+95)  

                theta[i] = radians(theta[i])
                motorID.set_position(tmp)
            
            fk()
            inputPos = user_input()

        loop = user_input()

    
    # disconnect
    my_dxl_2.disable_torque()
    my_dxl_3.disable_torque()
    my_dxl_4.disable_torque()
    my_dxl_5.disable_torque()

    Ax12.close_port()

if __name__ == '__main__':
    main()
