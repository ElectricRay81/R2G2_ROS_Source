#!/usr/bin/env python

######################################################################################################################################################
# State Control Script
# This script checks if R2G2 will be running in manual control, Go2Goal or Autonomous Control by toggle a button on the joystick
# 
# Counter value 0 = Manual control (Joystick)
# Counter value 1 = Go 2 Goal control  
# Counter value 2 = Autonomous control
#
# Ray Gilbers aug - 2023
######################################################################################################################################################

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from r2g2.msg import Control_State

class JoystickCounter:
    def __init__(self):
        # Create a node that counts the the pressed button
        rospy.init_node('joystick_counter')
        self.counter = 0
        rospy.loginfo("Counter: %d", self.counter)
        rospy.loginfo("Manual control")
        self.button_pressed = False
        self.publish_control_state = rospy.Publisher('/control_state', Control_State, queue_size=10)

    def joystick_callback(self, data):
        # Check if button[1] is pressed, debounce function
        if data.buttons[11] == 1:
            if not self.button_pressed:
                self.button_pressed = True
                self.increment_counter()
        else:
            self.button_pressed = False

    def increment_counter(self):
        # Set the counter value (from 0 to 2)
        self.counter = (self.counter + 1) % 3
        rospy.loginfo("Counter: %d", self.counter)
        
        if self.counter == 0:
            rospy.loginfo("Manual control")
        
        if self.counter == 1:
            rospy.loginfo("Go2Goal control")

        if self.counter == 2:
            rospy.loginfo("Autonomous control")

        # Publish the desired control state     
        control_state_msg = Control_State()
        control_state_msg.control_state_value = self.counter
        self.publish_control_state.publish(control_state_msg)
        

    def run(self):
        rospy.Subscriber('joy', Joy, self.joystick_callback)
        rospy.spin()

if __name__ == '__main__':   
    try:
        jc = JoystickCounter()
        jc.run()
    except rospy.ROSInterruptException:
        pass
