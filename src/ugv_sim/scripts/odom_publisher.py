
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf




class odom_publisher:
    def __init__(self):
        rospy.init_node("odom_publisher")
        rospy.Subscriber('/world', Odometry, self.odom_callback)


    def odom_callback(self, msg):
        transform_from = "/base_link"
        transform_to   = "/world"
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        orientation = [ori.x, ori.y, ori.z, ori.w]
        position    = [pos.x, pos.y, pos.z]
        br = tf.TransformBroadcaster()

        br.sendTransform(position, orientation, rospy.Time.now(), transform_from, transform_to)

if __name__ == '__main__':
    
    try:
        odom = odom_publisher()
        rospy.sleep(1.0)
        rate = 20.0
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
