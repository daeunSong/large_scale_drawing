#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

class OdomPublisher(object):
    def __init__(self):
        # Create a publisher for publishing the distance moved into the topic '/odom/filtered'
        self.odom_pub = rospy.Publisher('/odometry/filtered', Odometry, queue_size=1)
        self.listener = tf.TransformListener()
        # subscribe to odom topic
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        """Process odometry data sent by the subscriber."""
        # Get the position information from the odom message
        # See the structure of an /odom message in the `get_init_position` function
        odom_msgs = msg
        trans = None
        while trans is None:
            try:
                (trans, rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        odom_msgs.pose.pose.position.x = trans[0]
        odom_msgs.pose.pose.position.y = trans[1]
        odom_msgs.pose.pose.position.z = trans[2]
        odom_msgs.pose.pose.orientation.x = rot[0]
        odom_msgs.pose.pose.orientation.y = rot[1]
        odom_msgs.pose.pose.orientation.z = rot[2]
        odom_msgs.pose.pose.orientation.w = rot[3]

        self.odom_pub.publish(odom_msgs)

if __name__ == '__main__':
    # create a node for running the program
    rospy.init_node('odom_publisher_node', anonymous=True)
    # create an instance of the Odom Publisher
    odom_obj = OdomPublisher()
    # Keep the program running
    rospy.spin()
