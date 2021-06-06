# Snap-Pic

A 5 dof robotic arm functions as a self-calibrated Wefie robotic arm which would detect the centre position of a group and position itself to capture everyone in the group.

Manual control is provided for user to adjust their desired position. 

*\*This is a simple project with no control involved.*

![Image of Robotic Arm](https://github.com/angcx1997/snap-pic/blob/main/img/5dof_robot.PNG)

## Functionality
Motion
- Inverse kinematic(through geometry analysis) computes each joint angle to move the end effector to desired pose.
- Additional adjustments on orientation can be manual control though serial communication with ESP32 Node MCU integrated with WiFi module.(The code is not included)

Camera Self-Positioning
- Cascade classifier is used to detect human faces in the image captured by IP camera
- The camera will then self calibrated until the camera is centered at the midpoint of human faces

![Image of workflow](https://github.com/angcx1997/snap-pic/blob/main/img/workflow.PNG)
