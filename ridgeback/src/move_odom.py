#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

direction = (0, -1, 0) # x, y, z
linear_speed = 0.1 # 1 m/s
target_dist = 1.0 # 1 meter

def dist_callback(msg):
    """Callback function that processes messages from the subscriber."""
    # get the distance moved from the message
    distance_moved = msg.data

    # If distance is less than the target, continue moving the robot
    # Otherwise, stop it (by pubishing `0`)
    if msg.data < target_dist:
        move.linear.x = linear_speed * direction[0]
        move.linear.y = linear_speed * direction[1]
        move.linear.z = linear_speed * direction[2]

    if msg.data > target_dist:
        move.linear.x = 0
        move.linear.y = 0
        move.linear.z = 0

    pub.publish(move)

# create a node for running the program
rospy.init_node('move_odom')

# create a subscriber that gets the distance moved
sub = rospy.Subscriber('/moved_distance', Float64, dist_callback)

# Create a publisher that moves the robot
pub = rospy.Publisher('/cmd_vel', Twist, queue_size="1")

# Create a global variable for publising a Twist ("cmd_vel") message
move = Twist()

# Keep the program running
rospy.spin()
