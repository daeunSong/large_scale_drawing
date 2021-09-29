#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

x = 100
y = 100


def callback(msg):
    global x, y
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y


def go_to_goal(x_goal, y_goal):
    velocity_message = Twist()
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    print('start moving to ', x_goal, y_goal)

    i = 0
    while (True):
        i = i + 1

        K_linear = 0.05
        x_dist = x_goal - x
        y_dist = y_goal - y
        distance = math.hypot(x_dist, y_dist)

        velocity_message.linear.x = K_linear * (x_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.linear.y = K_linear * (y_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.angular.z = 0

        velocity_publisher.publish(velocity_message)
        if i% 20000 == 0:
            print(f'CUR: x: {round(x, 2)} y: {round(y, 2)} distance: {round(distance, 2)}')

        if (distance < 0.05):
            print('done')
            return True
    return False


if __name__ == "__main__":
    rospy.init_node('holonimoic_move_to_goal')
    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, callback)
    go_to_goal(0, 0)
