#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
import numpy as np

def map_value(value):
    # Maps a value from one range to another.
    original_min = 69
    original_max = 119
    target_min = -30
    target_max = 30
    return (value - original_min) / (original_max - original_min) * (target_max - target_min) + target_min

class LidarTFBroadcaster:
    def __init__(self):
        rospy.init_node('lidar_tf_broadcaster')  # Initialize ROS node
        
        # Broadcasters for static and dynamic TFs
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.dynamic_broadcaster = tf.TransformBroadcaster()

        # Subscriptions for lidar roll angle and base link transform
        rospy.Subscriber('/lidar_roll_angle', Float64, self.handle_lidar_roll)
        rospy.Subscriber('/base_link_transform', TransformStamped, self.handle_base_link_transform)

        # Initial states
        self.current_roll = 0.0
        self.base_link_transform = TransformStamped()
        self.initialize_base_link_transform()

        # Publish static transforms at startup
        self.send_static_transforms()
    
    def initialize_base_link_transform(self):
        # Initialize base_link transform with default values
        self.base_link_transform.header.stamp = rospy.Time.now()
        self.base_link_transform.header.frame_id = 'world'
        self.base_link_transform.child_frame_id = 'base_link'
        self.set_transform_to_zero(self.base_link_transform.transform)

    def set_transform_to_zero(self, transform):
        # Set translation and rotation of a transform to zero
        transform.translation.x = 0.0
        transform.translation.y = 0.0
        transform.translation.z = 0.0
        transform.rotation.x = 0.0
        transform.rotation.y = 0.0
        transform.rotation.z = 0.0
        transform.rotation.w = 1.0

    def angle2rad(self, angle):
        # Convert angle from degrees to radians
        return angle / 180.0 * np.pi
    
    def send_static_transforms(self):
        # Publishes static transforms for base footprint and lidar mount
        base_footprint = self.create_static_transform('base_link', 'base_footprint')
        lidar_itself = self.create_static_transform('lidar_link', 'laser', z=0.04)
        self.static_broadcaster.sendTransform([base_footprint, lidar_itself])
        
    def create_static_transform(self, parent_id, child_id, x=0.0, y=0.0, z=0.0):
        # Helper to create a static transform
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent_id
        transform.child_frame_id = child_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        q = tf.transformations.quaternion_from_euler(0, 0, 0)  # No rotation
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        return transform

    def handle_lidar_roll(self, msg):
        # Updates the current roll angle of the lidar based on the subscribed topic
        self.current_roll = map_value(msg.data) / 180.0 * np.pi

    def handle_base_link_transform(self, msg):
        # Updates the base_link's transform based on the subscribed topic
        self.update_base_link_transform(msg.transform)

    def update_base_link_transform(self, transform):
        # Updates the base_link transform with the received data
        q = tf.transformations.quaternion_from_euler(
            self.angle2rad(transform.rotation.x),
            self.angle2rad(transform.rotation.y),
            self.angle2rad(transform.rotation.z))
        self.base_link_transform.header.frame_id = 'world'
        self.base_link_transform.child_frame_id = 'base_link'
        self.base_link_transform.transform.translation = transform.translation
        self.base_link_transform.transform.rotation.x = q[0]
        self.base_link_transform.transform.rotation.y = q[1]
        self.base_link_transform.transform.rotation.z = q[2]
        self.base_link_transform.transform.rotation.w = q[3]

    def publish_dynamic_transforms(self):
        # Broadcasts dynamic TFs for lidar and base link
        q_lidar = tf.transformations.quaternion_from_euler(self.current_roll, 0, 0)
        self.dynamic_broadcaster.sendTransform((0, 0, 0.142), q_lidar, rospy.Time.now(), 'lidar_link', 'base_link')
        self.dynamic_broadcaster.sendTransform(
            (self.base_link_transform.transform.translation.x, self.base_link_transform.transform.translation.y, self.base_link_transform.transform.translation.z),
            (self.base_link_transform.transform.rotation.x, self.base_link_transform.transform.rotation.y, self.base_link_transform.transform.rotation.z, self.base_link_transform.transform.rotation.w),
            rospy.Time.now(), 'base_link', 'world')

    def run(self):
        # Main loop for the node
        rate = rospy.Rate(40.0)  # 40 Hz
        while not rospy.is_shutdown():
            self.publish_dynamic_transforms()
            rate.sleep()

if __name__ == '__main__':
    lidar_tf_broadcaster = LidarTFBroadcaster()
    lidar_tf_broadcaster.run()
