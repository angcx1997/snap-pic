from dxl_control.Ax12 import Ax12
import time
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

# connecting
Ax12.open_port()
Ax12.set_baudrate()


def user_input():
    """ Check to see if user wants to continue """
    ans = input('Continue? : y/n ')
    if ans == 'n':
        return False
    else:
        return True


# def test_pos(motor_object):
#     bool_test = True
#     while bool_test:
#         motor_object.get_position()
#         # desired angle input
#         input_pos = int(input("input pos: "))
#         motor_object.set_position(input_pos)
#         present_speed = motor_object.get_present_speed()
#         print('Present Speed:', present_speed)
#         motor_object.set_moving_speed(1023)
#         bool_test = user_input()

# test_pos(my_dxl)

# 0
my_dxl_1.set_moving_speed(50)
my_dxl_2.set_moving_speed(50)
my_dxl_3.set_moving_speed(50)
my_dxl_4.set_moving_speed(50)
my_dxl_5.set_moving_speed(50)

# while (1):
my_dxl_1.get_position()
my_dxl_2.get_position()
my_dxl_3.get_position()
my_dxl_4.get_position()
my_dxl_5.get_position()
# time.sleep(10)


# while True:
    
#     new_pos = 150
    
#     my_dxl_4.set_position(new_pos)
#     while my_dxl_4.is_moving():
#         print('IS MOVING:',my_dxl_4.is_moving())
#     break
# my_dxl_4.led_on()
# my_dxl_1.set_position(int((225/300)*1023))
# time.sleep(3)
# my_dxl_2.set_position(int((75/300)*1023))
# time.sleep(3)
# my_dxl_3.set_position(int((175/300)*1023))
# time.sleep(3)
# my_dxl_4.set_position(int((150/300)*1023))
# time.sleep(3)
# # time.sleep(5)

# my_dxl_1.set_position(int((225/300)*1023))
# time.sleep(3)
# my_dxl_3.set_position(0)
# time.sleep(3)
# my_dxl_3.set_position(int((175/300)*1023))
# time.sleep(3)
# my_dxl_4.set_position(int((150/300)*1023))
# time.sleep(3)

# disconnect

my_dxl_1.disable_torque()
my_dxl_2.disable_torque()
my_dxl_3.disable_torque()
my_dxl_4.disable_torque()
my_dxl_5.disable_torque()
Ax12.close_port()
