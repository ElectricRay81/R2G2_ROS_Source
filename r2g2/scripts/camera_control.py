#!/usr/bin/env python

######################################################################################################################################################
# Camera Control Script
# This script handles the input values from a joystick in order to control the camera built on R2G2. 
# The hat control will move the two servos on R2G2, hat fwd is moving the cam up, hat bwd is moving the cam down, hat left is moving the cam left and 
# hat right is moving the cam right.
# 
# Horizontal move 1 deg = Hat0x => axes[4]
# Horzontal move -10 deg = btn4 => buttons[4]
# Horizontal move +10 deg = btn5 => buttons[5]
# Vertical move 1 deg = Hat0Y => axes[5]
# Vertical move -10 deg = btn2 => buttons[2]
# Vertical move +10 deg = btn3 => buttons[3]
# Reset cam position, both servos at 90 deg = btn1 => axes[1]
#
# Ray Gilbers Aug - 2023
######################################################################################################################################################

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


vertCam = 0
horCam = 0

def joystick_callback(data):
    
    global vertCam
    global horCam

    vertCam = data.axes[5]
    horCam = data.axes[4]

    if data.buttons[4] == 1:
        horCam = 10

    if data.buttons[5] == 1:
        horCam = -10

    if data.buttons[2] == 1:
        vertCam = -10

    if data.buttons[3] == 1:
        vertCam = 10

    if data.buttons[1] == 1:
        horCam = 90
        vertCam = 90
    
    
    hor_cam_pos_pub.publish(horCam)
    vert_cam_pos_pub.publish(vertCam)

rospy.init_node("camera_control", anonymous=True)

hor_cam_pos_pub = rospy.Publisher("/hor_cam_pos", Float32, queue_size=5)
vert_cam_pos_pub = rospy.Publisher("/vert_cam_pos", Float32, queue_size=5)

# Create subscriber to joystick node
rospy.Subscriber('joy', Joy, joystick_callback)

rospy.spin()
