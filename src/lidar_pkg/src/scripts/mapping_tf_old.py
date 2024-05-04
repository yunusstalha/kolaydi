#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
import numpy as np


def map_value(value):
    # Original range
    original_min = 69
    original_max = 119
    # Target range
    target_min = -30
    target_max = 30
    
    # Mapping
    return (value - original_min) / (original_max - original_min) * (target_max - target_min) + target_min



class LidarTFBroadcaster:
    def __init__(self):
        # Initialize the node
        rospy.init_node('lidar_tf_broadcaster')

        # Static transform broadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Dynamic transform broadcaster for lidar_link
        self.dynamic_broadcaster = tf.TransformBroadcaster()

        # Subscriber for the lidar roll angle
        rospy.Subscriber('/lidar_roll_angle', Float64, self.handle_lidar_roll)
        rospy.Subscriber('/base_link_transform', TransformStamped, self.handle_base_link_transform)



        # Current roll angle (initialize to zero)
        self.current_roll = 0.0
        self.base_link_transform = TransformStamped()
        self.base_link_transform.header.stamp = rospy.Time.now()
        self.base_link_transform.header.frame_id = 'world'
        self.base_link_transform.child_frame_id = 'base_link'
        self.base_link_transform.transform.translation.x = 0.0
        self.base_link_transform.transform.translation.y = 0.0
        self.base_link_transform.transform.translation.z = 0.0
        self.base_link_transform.transform.rotation.x = 0.0
        self.base_link_transform.transform.rotation.y = 0.0
        self.base_link_transform.transform.rotation.z = 0.0
        self.base_link_transform.transform.rotation.w = 1.0
        
        # Static Transforms
        self.send_static_transforms()
    
    def angle2rad(self, angle):
        return angle / 180.0 * np.pi
    
    def send_static_transforms(self):
        # Base link to Base Footprint
        base_footprint = TransformStamped()
        base_footprint.header.stamp = rospy.Time.now()
        base_footprint.header.frame_id = 'base_link'
        base_footprint.child_frame_id = 'base_footprint'
        base_footprint.transform.translation.x = 0.0
        base_footprint.transform.translation.y = 0.0
        base_footprint.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        base_footprint.transform.rotation.x = q[0]
        base_footprint.transform.rotation.y = q[1]
        base_footprint.transform.rotation.z = q[2]
        base_footprint.transform.rotation.w = q[3]

        # Lidar link to Lidar Itself
        lidar_itself = TransformStamped()
        lidar_itself.header.stamp = rospy.Time.now()
        lidar_itself.header.frame_id = 'lidar_link'
        lidar_itself.child_frame_id = 'laser'
        lidar_itself.transform.translation.x = 0.0
        lidar_itself.transform.translation.y = 0.0
        lidar_itself.transform.translation.z = 0.04
        q = tf.transformations.quaternion_from_euler(0, 0, 0)
        lidar_itself.transform.rotation.x = q[0]
        lidar_itself.transform.rotation.y = q[1]
        lidar_itself.transform.rotation.z = q[2]
        lidar_itself.transform.rotation.w = q[3]



        # Send the static transforms
        self.static_broadcaster.sendTransform([base_footprint, lidar_itself])
        # Send the static transform

        
    def handle_lidar_roll(self, msg):
        # Update the current roll angle
        self.current_roll = map_value(msg.data) / 180.0 * np.pi

    def handle_base_link_transform(self, msg):
        # Update the current transform for base_link
        q = tf.transformations.quaternion_from_euler(
            self.angle2rad(msg.transform.rotation.x),
            self.angle2rad(msg.transform.rotation.y),
            self.angle2rad(msg.transform.rotation.z))

        # self.base_link_transform.header.stamp = rospy.Time.now()
        self.base_link_transform.header.frame_id = 'world'
        self.base_link_transform.child_frame_id = 'base_link'

        self.base_link_transform.transform.translation.x = msg.transform.translation.x
        self.base_link_transform.transform.translation.y = msg.transform.translation.y
        self.base_link_transform.transform.translation.z = msg.transform.translation.z

        self.base_link_transform.transform.rotation.x = q[0]
        self.base_link_transform.transform.rotation.y = q[1]
        self.base_link_transform.transform.rotation.z = q[2]
        
        self.base_link_transform.transform.rotation.w = q[3]


    def publish_dynamic_transforms(self):
        # Broadcast the dynamic transform for lidar_link
        q = tf.transformations.quaternion_from_euler(self.current_roll, 0, 0)
        self.dynamic_broadcaster.sendTransform(
            (0, 0, 0.142),  # translation
            q,  # rotation
            rospy.Time.now(),
            'lidar_link',  # child frame
            'base_link'  # parent frame
        )
        # Broadcast the dynamic transform for base_link
        self.dynamic_broadcaster.sendTransform(
            (self.base_link_transform.transform.translation.x,
             self.base_link_transform.transform.translation.y,
             self.base_link_transform.transform.translation.z),

            (self.base_link_transform.transform.rotation.x,
             self.base_link_transform.transform.rotation.y,
             self.base_link_transform.transform.rotation.z,
             self.base_link_transform.transform.rotation.w),

            rospy.Time.now(),
            'base_link',
            'world'  # parent frame is now 'world'
        )


    def run(self):
        # Main loop
        rate = rospy.Rate(40.0)
        while not rospy.is_shutdown():
            self.publish_dynamic_transforms()
            rate.sleep()

if __name__ == '__main__':
    lidar_tf_broadcaster = LidarTFBroadcaster()
    lidar_tf_broadcaster.run()