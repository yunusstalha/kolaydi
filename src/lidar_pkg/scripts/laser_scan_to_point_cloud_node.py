#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
import laser_geometry
import tf

class LaserScanToPointCloud:
    def __init__(self):
        # Initialize the node, subscribers, and publishers
        self.node_name = "laser_scan_to_point_cloud"
        rospy.init_node(self.node_name)

        # Subscribe to laser scans and a state control topic
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.state_sub = rospy.Subscriber("/state_control", String, self.state_callback)

        # Publisher for the converted point cloud data
        self.cloud_pub = rospy.Publisher("/cloud", PointCloud2, queue_size=10)

        # Tools for laser scan projection and TF transformations
        self.laser_projector = laser_geometry.LaserProjection()
        self.tf_listener = tf.TransformListener()

        # Initial state of the node
        self.state = "MOVE"
        rospy.loginfo("State changed to MOVE")

    def laser_callback(self, data):
        # Callback for processing laser scan data
        if self.state == "MAP":
            # Convert and publish point cloud only in the 'MAP' state
            try:
                cloud = self.laser_projector.projectLaser(data)
                self.cloud_pub.publish(cloud)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                # Handle any TF related exceptions
                pass

    def state_callback(self, msg):
        # Update operating state based on incoming messages
        self.state = msg.data
        if self.state in ["MAP", "MOVE"]:
            rospy.loginfo("State changed to " + self.state)
        else:
            rospy.loginfo("Invalid state, please enter either MAP or MOVE")

if __name__ == '__main__':
    # Initialize and run the ROS node
    node = LaserScanToPointCloud()
    rospy.spin()
