#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import numpy as np
import math

import time
from std_msgs.msg import String

iiwa_done = 0

class MoveOdom (object):
    def __init__(self):
        """Initialize an object of the MoveOdom class."""
        # state publisher
        ir_pub = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback', String, queue_size=100)
        
        # Target direction and distance in 2D referenced to CURRENT postition
        self.linear_speed = 0.1 #0.01 # 1 m/s
        self.direction = np.array([0, 1]) # (0, -1) is moving left
        self.target_dist = 0.45 # in meter
        self.target_position = self.direction * self.target_dist

        # Create a global variable for publising a Twist ("cmd_vel") message
        self.move = Twist()
        self.reached = False

        # Get the inital position. This will be a reference point for calculating
        # the distance moved
        self.get_init_position()

        # Create a publisher that moves the robot
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size="1")
        # create a subscriber for getting new Odometry messages
        rospy.Subscriber("/odometry/filtered", Odometry, self.odom_callback)
        rospy.Subscriber('/iiwa_ridgeback_communicaiton/iiwa', String, self.get_communication)

    def get_init_position(self):
        """Get the initial position of the robot."""
        """
        Structure of the odom position message:
        user:~$ rostopic echo /odom -n1
        header:
        seq: 14929
        stamp:
            secs: 748
            nsecs: 215000000
        frame_id: "odom"
        child_frame_id: "base_footprint"
        pose:
        pose:
            position:
            x: 0.00668370211388
            y: 0.00010960687178
            z: -0.000246865753431
        """
        data_odom = None
        # wait for a message from the odometry topic and store it in data_odom when available
        while data_odom is None:
            try:
                data_odom = rospy.wait_for_message("/odometry/filtered", Odometry, timeout=1)
                ### real robot working code
                # data_odom = rospy.wait_for_message("/ridgeback_velocity_controller/odom", Odometry, timeout=1)
            except:
                rospy.loginfo("Current odom not ready yet, retrying for setting up init pose")

        # Store the received odometry "position" variable in a Point instance
        self._current_position = Point()
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z
        self.target_position += np.array([data_odom.pose.pose.position.x, data_odom.pose.pose.position.y])

    def get_communication(self, msg):
        global iiwa_done
        iiwa_done = int(msg.data)
        print("iiwa said: ", iiwa_done)
    
    def odom_callback(self, msg):
        """Callback function that processes messages from the subscriber."""
        # get the distance moved from the message
        new_position = msg.pose.pose.position

        # If distance is less than the target, continue moving the robot
        # Otherwise, stop it (by pubishing `0`)
        dist = self.calculate_distance(self.target_position, new_position)
        if (iiwa_done and not self.reached):
            if dist > 0.001:
                self.move.linear.x = self.linear_speed * self.direction[0]
                self.move.linear.y = self.linear_speed * self.direction[1]

            if dist < 0.001:
                self.move.linear.x = 0
                self.move.linear.y = 0
                self.reached = True
                self.ir_pub.publish("3")

        self.move_pub.publish(self.move)

    def calculate_distance(self, target_position, new_position):
        """Calculate the distance between two Points (positions)."""
        x2 = new_position.x
        x1 = target_position[0]
        y2 = new_position.y
        y1 = target_position[1]
        dist = math.hypot(x2 - x1, y2 - y1)
        return dist

if __name__ == '__main__':
    # create a node for running the program
    rospy.init_node('move_odom_tf', anonymous=True)
    # create an instance of the MoveOdom
    odom_obj = MoveOdom()
    # Keep the program running
    rospy.spin()

