#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String 
import laser_geometry
import tf
import tf2_ros  
import sensor_msgs.point_cloud2 as pc2 
from tf2_geometry_msgs import PointStamped


class LaserScanToPointCloud:
    def __init__(self):
        self.node_name = "laser_scan_to_point_cloud"
        rospy.init_node(self.node_name)

        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cloud_pub = rospy.Publisher("/cloud", PointCloud2, queue_size=10)
        self.state_sub = rospy.Subscriber("/state_control", String, self.state_callback)

        self.laser_projector = laser_geometry.LaserProjection()
        self.tf_listener = tf.TransformListener()
        self.tf_buffer = tf2_ros.Buffer()  
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer) 
        self.filter_threshold = -0.0  # Threshold for filtering points below base_link
        self.state = "MOVE"  
        rospy.loginfo("State changed to MOVE")
        
    def laser_callback(self, data):
        if self.state == "MAP":
            try:
                cloud = self.laser_projector.projectLaser(data)
                transform = self.tf_buffer.lookup_transform(
                    "base_link", data.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
                )
                self.publish_filtered_cloud(cloud, transform,data)

            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def publish_filtered_cloud(self, cloud, transform, data):
        generator = pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
        filtered_points = []
        for p in generator:
            x, y, z = p[0], p[1], p[2]

            # Create a PointStamped object
            point_stamped = PointStamped()
            point_stamped.header.frame_id = data.header.frame_id 
            point_stamped.point.x = x
            point_stamped.point.y = y
            point_stamped.point.z = z
            
            # Apply transform using PointStamped
            transformed_point_stamped = self.tf_buffer.transform(point_stamped, "base_link")

            # Access the transformed coordinates
            transformed_point = [
                transformed_point_stamped.point.x,
                transformed_point_stamped.point.y,
                transformed_point_stamped.point.z,
            ]

            # Check if point is below threshold
            if transformed_point[2] > self.filter_threshold:
                filtered_points.append(transformed_point)

        # Create new PointCloud2 message
        filtered_cloud = pc2.create_cloud_xyz32(cloud.header, filtered_points)
        self.cloud_pub.publish(filtered_cloud)

    def state_callback(self, msg):
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