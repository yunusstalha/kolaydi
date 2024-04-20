#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def cmd_vel_callback(data):
    # Example conversion for a differential drive robot
    linear_speed = data.linear.x
    angular_speed = data.angular.z
    
    # Assuming a robot with a certain wheel base
    wheel_base = 0.2 # meters
    right_wheel_speed = linear_speed + (wheel_base / 2.0) * angular_speed
    left_wheel_speed = linear_speed - (wheel_base / 2.0) * angular_speed
    
    # Code to send these speeds to your robot's motors would go here
    # This could involve publishing to another topic, or interfacing with hardware directly

def listener():
    rospy.init_node('diff_drive_listener', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
