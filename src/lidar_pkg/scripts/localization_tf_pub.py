#!/usr/bin/env python
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64

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

        # Static Transforms
        self.send_static_transforms()

        # Current roll angle (initialize to zero)
        self.current_roll = 0.0

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
        self.current_roll = msg.data / 180.0 * 3.14159 - 3.14159 / 2.0

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

    def run(self):
        # Main loop
        rate = rospy.Rate(40.0)
        while not rospy.is_shutdown():
            self.publish_dynamic_transforms()
            rate.sleep()

if __name__ == '__main__':
    lidar_tf_broadcaster = LidarTFBroadcaster()
    lidar_tf_broadcaster.run()