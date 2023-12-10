#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String  # Import String message type for state control
import laser_geometry
import tf

class LaserScanToPointCloud:
    def __init__(self):
        self.node_name = "laser_scan_to_point_cloud"
        rospy.init_node(self.node_name)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cloud_pub = rospy.Publisher("/cloud", PointCloud2, queue_size=10)
        self.state_sub = rospy.Subscriber("/state_control", String, self.state_callback)  # State control subscriber

        self.laser_projector = laser_geometry.LaserProjection()
        self.tf_listener = tf.TransformListener()

        self.state = "MOVE"  # Default state

    def laser_callback(self, data):
        # Process and publish the point cloud only if in MAP state
        if self.state == "MAP":
            try:
                cloud = self.laser_projector.projectLaser(data)
                self.cloud_pub.publish(cloud)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def state_callback(self, msg):
        # Update the state based on the received message
        self.state = msg.data
        if self.state == "MAP":
            rospy.loginfo("State changed to MAP")
        elif self.state == "MOVE":
            rospy.loginfo("State changed to MOVE")
        else:
            rospy.loginfo("Invalid state, please enter either MAP or MOVE")

if __name__ == '__main__':
    node = LaserScanToPointCloud()
    rospy.spin()
