#!/usr/bin/env python3

import math
import rospy
import time
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

x = 0
y = 0
w = 0

received_message = {}
send_message = {}

'''
number: instruction number
status: executing, done, error, waiting
==============
4 done
'''


def get_current_position(msg):
    global x, y, w
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w = msg.pose.pose.orientation.w


def get_communication(msg):
    global received_message
    msg_data = msg.split()
    received_message['number'] = int(msg_data[0])
    received_message['status'] = msg_data[1]


def go_to_goal(x_goal, y_goal, ir_pub):
    velocity_message = Twist()

    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    print('start moving to ', x_goal, y_goal)

    i = 0
    send_message['status'] = 'executing'
    ir_pub.publish(f'{send_message["number"]} {send_message["status"]} ')
    while (True):

        i += 1

        K_linear = 0.05
        x_dist = x_goal - x
        y_dist = y_goal - y
        distance = math.hypot(x_dist, y_dist)

        velocity_message.linear.x = K_linear * (x_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.linear.y = K_linear * (y_dist / (abs(x_dist) + abs(y_dist)))
        velocity_message.angular.z = 0
        velocity_publisher.publish(velocity_message)

        if i % 15000 == 0:
            print(
                f'CUR: x: {round(x, 2)} y: {round(y, 2)} distance: {round(distance, 2)} x_dist: {round(x_dist, 2)} y_dist: {round(y_dist, 2)}')

        if (distance < 0.05):
            print('done')
            return True

    return False


def with_iiwa_follow_traj(path, ir_pub):
    global send_message
    send_message['number'] = 0

    for (px, py) in path:
        send_message['number'] += 1
        send_message['status'] = 'started'
        ir_pub.publish(f'{send_message["number"]} {send_message["status"]}')

        result = go_to_goal(px, py, ir_pub)

        if result:

            send_message['status'] = 'done'
            ir_pub.publish(f'{send_message["number"]} {send_message["status"]}')
            rospy.loginfo("Goal execution done!")

            # while received_message['executed'] != 'done':
            send_message['status'] = 'waiting'
            ir_pub.publish(f'{send_message["number"]} {send_message["status"]}')
                # rospy.loginfo(f'iiwa is done {recieved_message["executed"]} now starting {send_message["executed"]+1}')
        else:
            pass
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        rospy.init_node('drawing_ridgeback')

        odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, get_current_position)

        ir_sub = rospy.Subscriber('/iiwa_ridgeback_communicaiton/iiwa', String, get_communication)
        ir_pub = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback', String, queue_size=10)

        rospy.sleep(0.5)

        # x_positive = [x for x in x_points]
        # y_positive = [abs(y) - 0.8 for y in y_points]

        x_positive = [0, 1, 2]
        y_positive = [0, 1, 2]

        path = zip(x_positive, y_positive)

        with_iiwa_follow_traj(path, ir_pub)

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
