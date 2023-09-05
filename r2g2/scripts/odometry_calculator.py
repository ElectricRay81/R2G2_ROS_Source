#!/usr/bin/env python

######################################################################################################################################################
# Position Computation
# This script handles takes the left and right motor speeds in rpm and compute a linear and angular velocity of R2G2
# 
#
# Ray Gilbers Aug - 2023
######################################################################################################################################################

import rospy
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf

class OdometryCalculator:
    # Initialization function
    def __init__(self):
        rospy.init_node('odometry_calculator')

        # Initialize starting coordinates
        self.x = 0
        self.y = 0
        self.yaw = 0

        # Initialie starting time
        self.last_time = rospy.Time.now()

        # Create subscribers and publisher
        self.sub_right_vel = rospy.Subscriber('/right_velocity', Float32, self.right_vel_callback)
        self.sub_left_vel = rospy.Subscriber('/left_velocity', Float32, self.left_vel_callback)
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size = 10)

        self.tf_broadcaster = tf.TransformBroadcaster()

    def publish_tf(self):
            self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),  # Translation
            tf.transformations.quaternion_from_euler(0, 0, self.yaw),  # Rotation
            rospy.Time.now(),
            'base_link',
            'odom'
        )
            
            # Add transformation from base_footprint to base_link
            self.tf_broadcaster.sendTransform(
                (0, 0, 0),  # No translation offset between base_footprint and base_link
                (0, 0, 0, 1),  # No rotation offset (identity quaternion)
                rospy.Time.now(),
                'base_link',
                'base_footprint'
        )
    
    def calculate_odometry(self, right_vel_rpm, left_vel_rpm):
        wheelbase = 0.205                                                                                   # Wheelbase in m
        wheel_radius = 0.036                                                                                # Radius of wheel in m

        right_vel_mps = (2 * math.pi * wheel_radius * right_vel_rpm) / 60.0                                 # Convert rpm to m/s right side
        left_vel_mps = (2 * math.pi * wheel_radius * left_vel_rpm) / 60.0                                   # Convert rpm to m/s left side

        linear_velocity = (right_vel_mps + left_vel_mps) * 0.5                                              # Compute the linear velocity
        angular_velocity = (right_vel_mps - left_vel_mps) / wheelbase                                       # Compute angular velocity in rad/s

        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()                                                       # Compute integration time and covert from us to sec

        self.x += linear_velocity * math.cos(self.yaw) * dt
        self.y += linear_velocity * math.sin(self.yaw) * dt
        self.yaw += angular_velocity * dt

        self.publish_odom()
        self.publish_tf()


        self.last_time = current_time

    def right_vel_callback(self, msg):
        self.right_vel_rpm = msg.data
        self.calculate_odometry(self.right_vel_rpm, self.left_vel_rpm)
    
    def left_vel_callback(self, msg):
        self.left_vel_rpm = msg.data
        self.calculate_odometry(self.right_vel_rpm, self.left_vel_rpm)

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Populate position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # Populate velocities
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.pub_odom.publish(odom)

if __name__ == '__main__':
    try:
        odometry_calculator = OdometryCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass




