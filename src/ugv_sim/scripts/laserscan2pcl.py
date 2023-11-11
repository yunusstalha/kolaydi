#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry
import tf

class LaserScanToPointCloud:
    def __init__(self):
        self.node_name = "laser_scan_to_point_cloud"
        rospy.init_node(self.node_name)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cloud_pub = rospy.Publisher("/cloud", PointCloud2, queue_size=10)

        self.laser_projector = laser_geometry.LaserProjection()
        self.tf_listener = tf.TransformListener()

    def laser_callback(self, data):
        try:
            # Transform the laser scan into a point cloud
            cloud = self.laser_projector.projectLaser(data)
            # Publish the point cloud
            self.cloud_pub.publish(cloud)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

if __name__ == '__main__':
    node = LaserScanToPointCloud()
    rospy.spin()
