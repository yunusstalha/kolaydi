# #!/usr/bin/env python
# import rospy
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
# import math

# class RobotPositionController:
#     def __init__(self):
#         # Initialize the node
#         rospy.init_node('robot_position_controller')
        
#         # Publisher to send velocity commands to the robot's base
#         self.cmd_vel_pub = rospy.Publisher('/controller1_name/cmd_vel', Twist, queue_size=10)
        
#         # Subscriber to get the current position and orientation of the robot
#         rospy.Subscriber('/controller1_name/odom', Odometry, self.odom_callback)
        
#         # Robot's current position and orientation
#         self.current_position = [0.0, 0.0]  # x, y
#         self.current_orientation = 0.0  # yaw
        
#         # Desired position for the robot
#         self.desired_position = [1.0, 1.0]  # x, y
#         self.desired_orientation = 0.0  # yaw (optional, if you want to control orientation)


#     def odom_callback(self, msg):
#         # Update the current position of the robot
#         self.current_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
#         # Get the current orientation from the quaternion
#         orientation_q = msg.pose.pose.orientation
#         orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#         (_, _, yaw) = euler_from_quaternion(orientation_list)
#         self.current_orientation = yaw
    
#     def control_loop(self):
#         rate = rospy.Rate(10)  # 10 Hz
#         while not rospy.is_shutdown():
#             # Compute the control action to reach the desired position
#             control_action = Twist()
            
#             # Compute your control law here to populate control_action
#             # This is where you implement your position control algorithm
#             # For example, a simple proportional controller could be:
#             error_x = self.desired_position[0] - self.current_position[0]
#             error_y = self.desired_position[1] - self.current_position[1]
            
#             # Convert error to robot's frame of reference
#             error_forward = error_x * math.cos(self.current_orientation) + error_y * math.sin(self.current_orientation)
#             error_lateral = -error_x * math.sin(self.current_orientation) + error_y * math.cos(self.current_orientation)
            
#             # Simple proportional control
#             control_action.linear.x = 0.5 * error_forward
#             control_action.angular.z = 0.1 * error_lateral  # Adjust this if you also want to control orientation
            
#             # Ensure the velocity commands are within bounds
#             # ...

#             # Publish the velocity commands
#             self.cmd_vel_pub.publish(control_action)
#             rospy.loginfo("Current position: %s", self.current_position)
#             rospy.loginfo("Desired position: %s", self.desired_position)
#             rospy.loginfo("Error: %s", [error_x, error_y])
#             rate.sleep()

# if __name__ == '__main__':
#     try:
#         controller = RobotPositionController()
#         controller.control_loop()
#     except rospy.ROSInterruptException:
#         pass





# def forward_kinematics(self, v_left_wheel, v_right_wheel, theta, dt):
#     # Calculate linear and angular velocities
#     v = (v_left_wheel + v_right_wheel) / 2
#     w = (v_right_wheel - v_left_wheel) / self.wheel_base
    
#     # Update pose
#     delta_x = v * np.cos(theta) * dt
#     delta_y = v * np.sin(theta) * dt
#     delta_theta = w * dt

#     return delta_x, delta_y, delta_theta


# def jacobian(self, v_left_wheel, v_right_wheel, theta):
#     # Calculate elements of the Jacobian matrix
#     dx_dvl = np.cos(theta) / 2
#     dx_dvr = np.cos(theta) / 2
#     dw_dvl = 1 / (2 * self.wheel_base)
#     dw_dvr = -1 / (2 * self.wheel_base)

#     return np.array([[dx_dvl, dx_dvr], [dw_dvl, dw_dvr]])


#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
import tf
import numpy as np


class RobotPositionController:
    def __init__(self):
        # Robot properties
        self.wheel_radius = 0.06  # Radius of the robot's wheels


        rospy.init_node('robot_position_controller')
        self.cmd_vel_pub = rospy.Publisher('/controller1_name/cmd_vel', Twist, queue_size=10)
        self.right_wheel_pub = rospy.Publisher('/right_wheel_controller/command', Float64, queue_size=10)
        self.left_wheel_pub = rospy.Publisher('/left_wheel_controller/command', Float64, queue_size=10)
        
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/clicked_point',PointStamped , self.clicked_point_callback)
        self.current_position = [0.0, 0.0]
        self.current_orientation = 0.0
        

        self.desired_position = [0, 14.22]
        
        # Robot parameters
        self.wheel_base = 0.3  # distance between the wheels
        
        # PID control parameters for linear position control
        self.kp_linear = 5  # proportional gain
        self.ki_linear = 0.0  # integral gain
        self.kd_linear = 0.01 # derivative gain
        self.linear_integral = 0.0
        self.last_linear_error = 0.0
        
        # PID control parameters for angular position control
        self.kp_angular = 8.  # proportional gain
        self.ki_angular = 0.  # integral gain
        self.kd_angular = 0.1 # derivative gain
        self.angular_integral = 0.0
        self.last_angular_error = 0.0
        
        # Control loop timing
        self.last_time = rospy.Time.now()

    def diff_drive(self, linear_velocity, angular_velocity):
        # Calculate wheel velocities
        v_left_wheel = linear_velocity - angular_velocity * self.wheel_base / 2
        v_right_wheel = linear_velocity + angular_velocity * self.wheel_base / 2
        return v_left_wheel, v_right_wheel

    def odom_callback(self, msg):
        # Extract position and orientation from odometry message
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)


        transform_from = "/base_link"
        transform_to   = "odom"
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        orientation = [ori.x, ori.y, ori.z, ori.w]
        position    = [pos.x, pos.y, pos.z]
        br = tf.TransformBroadcaster()

        br.sendTransform(position, orientation, rospy.Time.now(), transform_from, transform_to)

        self.current_orientation = yaw
    def clicked_point_callback(self, msg):
        # Extract position and orientation from odometry message
        self.desired_position[0] = msg.point.x
        self.desired_position[1] = msg.point.y
        # orientation_q = msg.pose.pose.orientation
        # orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # (_, _, yaw) = euler_from_quaternion(orientation_list)
        # self.desired_orientation = yaw

    def control_loop(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()

            # Calculate error in the robot's frame of reference
            delta_x = self.desired_position[0] - self.current_position[0]
            delta_y = self.desired_position[1] - self.current_position[1]
            delta_theta = math.atan2(delta_y, delta_x) - self.current_orientation
            
            # Wrap delta_theta to [-pi, pi]
            delta_theta = math.atan2(math.sin(delta_theta), math.cos(delta_theta))
            
            distance = math.sqrt(delta_x**2 + delta_y**2)
            linear_error = distance
            angular_error = delta_theta
            
            # Update the integral and derivative errors for linear control
            self.linear_integral += linear_error * dt
            linear_derivative = (linear_error - self.last_linear_error) / dt
            
            # Update the integral and derivative errors for angular control
            self.angular_integral += angular_error * dt
            angular_derivative = (angular_error - self.last_angular_error) / dt
            
            # Compute PID control signals
            linear_velocity = (self.kp_linear * linear_error +
                               self.ki_linear * self.linear_integral +
                               self.kd_linear * linear_derivative)
            
            angular_velocity = (self.kp_angular * angular_error +
                                self.ki_angular * self.angular_integral +
                                self.kd_angular * angular_derivative)
            
            # Construct the velocity command
            control_signal = Twist()
            control_signal.linear.x = max(min(linear_velocity, 15.0), -15.0)  # clamp to [-1, 1]
            control_signal.angular.z = max(min(angular_velocity, 15.0), -15.0)  # clamp to [-1, 1]
            
            # Publish the velocity command
            vel_left, vel_right = Float64(), Float64()
            
            vel_left.data, vel_right.data = self.diff_drive(control_signal.linear.x, control_signal.angular.z)
            self.right_wheel_pub.publish(vel_right)
            self.left_wheel_pub.publish(vel_left)
            #self.cmd_vel_pub.publish(control_signal)
            
            # Save the current errors and time for the next iteration
            self.last_linear_error = linear_error
            self.last_angular_error = angular_error
            self.last_time = current_time

            rospy.loginfo("Wheel Velocity: %s", [vel_right, vel_left])
            rospy.loginfo("Current Velocity: %s", control_signal.linear.x)
            rospy.loginfo("Current position: %s", self.current_position)
            rospy.loginfo("Desired position: %s", self.desired_position)
            
            rospy.loginfo("Error: %s", [delta_x, delta_y])
            rospy.loginfo("Angular error: %s", angular_error)
            if abs(delta_x) < 0.01 and abs(delta_y) < 0.01:
                rospy.loginfo("Reached the goal!")
                vel_left.data, vel_right.data = 0, 0
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = RobotPositionController()
        rospy.sleep(1.0)
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
