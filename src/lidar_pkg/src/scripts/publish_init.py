import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def set_initial_pose():
    rospy.init_node('set_initial_pose', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.sleep(1)  # Wait for the publisher to establish connection to the topic

    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.position.x = 0  # Set your x coordinate
    initial_pose.pose.pose.position.y = 0  # Set your y coordinate
    initial_pose.pose.pose.orientation.w = 1.0  # Assuming the robot is facing forward
    # Set a smaller covariance (diagonal elements)
    initial_pose.pose.covariance[0] = 0.1  # Variance for x
    initial_pose.pose.covariance[7] = 0.1  # Variance for y
    initial_pose.pose.covariance[35] = 0.2  # Variance for yaw

    pub.publish(initial_pose)
    print("Initial pose set.")

if __name__ == '__main__':
    try:
        set_initial_pose()
    except rospy.ROSInterruptException:
        pass
