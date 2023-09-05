#!/usr/bin/env python

######################################################################################################################################################
# Control Input Script
# This script handles the input values from a joystick in order to control R2G2 manually
# 
#
# Ray Gilbers July - 2023
######################################################################################################################################################

import rospy
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from r2g2.msg import Control_State


# Global variables used for processing the input value from the joystick
Xvelocity = 0
Yvelocity = 0
Zvelocity = 0
Max_Velocity = 100                              # Max velocity in rpm

# Global variables which will be published for left and right motor speed
Left_Motor_Speed = 0
Right_Motor_Speed = 0
Rotation_Speed = 0

# Initialize control input node
rospy.init_node("control_input", anonymous=True)

# Callback funtion for the Control State of R2G2 depending on the value a different control will be accomplished
def control_state_callback(data):
    rospy.loginfo("Current control state is: %d", data.control_state_value)
    print(data.control_state_value)

def joystick_callback(data):
# Get joystick values max scale from joystick is -1 to 1 for x, y and z axis
    axes = data.axes
    global Yvelocity                             # Y position joystick represents Forward and Backwards velocity
    global Xvelocity                             # X position joystick represents turn angle will be used for difference in motor speed
    #global Zvelocity                             # Z position joystick represents rotation around Bot axis
    #Zvelocity = axes[2] * Max_Velocity           # Scale joystick value to rpm value   
    Yvelocity = axes[1] * Max_Velocity           # Scale joystick value to rpm value
    Xvelocity = axes[0] * Max_Velocity           # Scale joystick value to rpm value   
    process_velocities(Xvelocity, Yvelocity)
    
def process_velocities(x_val, y_val):
# First check if x and y values are not equal to zero
    if x_val == 0:
        # If the X position of the joystick is 0 than the bot drives straight and not further computation has to be done
        # Therefore both motors run equal speed.
        Left_Motor_Speed = y_val
        Right_Motor_Speed = y_val
        print("Left speed: ")
        print(Left_Motor_Speed)
    else:
        hyp = math.sqrt(x_val**2 + y_val**2)                # Find the hypothenuse of the two vectors
        vel_factor = abs(y_val/hyp)                         # Compute the cosine and use it as a attenuation factor for one of the wheels
        if x_val > 0:                                       # If x pos joystick is > 0, R2G2 needs to move to left, left motor turns slower as right motor
            Left_Motor_Speed = vel_factor * y_val
            Right_Motor_Speed = y_val
        else:                                               # If x pos joystick is < 0, R2G2 needs to move to right, right motor turns slower as left motor
            Left_Motor_Speed = y_val
            Right_Motor_Speed = vel_factor * y_val   
    

    # Publish the computed velocities for each motor
    left_vel_pub.publish(Left_Motor_Speed)
    right_vel_pub.publish(Right_Motor_Speed)

# Create subscriber to Joystick node 
def joystick_subscriber():
    rospy.Subscriber('joy', Joy, joystick_callback)
    rospy.spin()

# Create Velocity publishers
left_vel_pub = rospy.Publisher("/left_velocity", Float32, queue_size=25)
right_vel_pub = rospy.Publisher("/right_velocity", Float32, queue_size=25)
# rotation_vel_pub = rospy.Publisher("/rotation_velocity", Float32, queue_size=10 )

while not rospy.is_shutdown():
    rospy.Subscriber("/control_state", Control_State, control_state_callback)
    joystick_subscriber()
    
