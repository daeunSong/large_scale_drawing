#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import numpy as np
import tools


class Tracker:
    def __init__(self, name = '515D3307'):
        self.tracker_name = name
        self.pose = None
        # tracker calibration
        self.origin = None
        self.x_axis = [1, 0, 0]
        self.y_axis = [0, 0, 1]
        self.rot_x = None
        self.rot_M = None
        self.rot_q = None

class Calibration:
    def __init__(self):
        self.tracker = Tracker('515D3307')
        self.wall = Tracker('00F03808')

        self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.publisher_tracker = rospy.Publisher('/vive/tracker', Odometry, queue_size=10)
        self.publisher_wall = rospy.Publisher('/vive/wall', Odometry, queue_size=10)
        self.subscriber_tracker = rospy.Subscriber('/vive/LHR_'+self.tracker.tracker_name+'_pose', PoseWithCovarianceStamped, self.tracker_callback)
        self.subscriber_wall = rospy.Subscriber('/vive/LHR_'+self.wall.tracker_name+'_pose', PoseWithCovarianceStamped, self.wall_callback)
        self.subscriber_odom = rospy.Subscriber('/ridgeback_velocity_controller/odom', Odometry, self.odom_callback, queue_size=1)
        
        self.linear_speed = 0.05
        self.angular_speed = 0.02
        self.r = 0.0

        self.odom_yaw = None
        self.odom_pose = None


    def odom_callback(self, msg):
        """Return real robot odometry"""
        self.odom_pose = msg.pose.pose
        qaut = np.array([self.odom_pose.orientation.x,self.odom_pose.orientation.y,self.odom_pose.orientation.z,self.odom_pose.orientation.w])
        self.odom_yaw = R.from_quat(qaut).as_euler('zyx')[0]

    def tracker_callback(self, msg):
        """Return vive tracker value"""
        self.tracker.pose = msg.pose.pose

    def wall_callback(self, msg):
        """Return wall tracker value only once"""
        self.wall.pose = msg.pose.pose

    def move_relative(self, target_position):
        """Command Ridgeback to move to relative position via odometry"""
        # set direction
        direction = np.array(target_position)
        direction = direction / np.linalg.norm(direction)
        # set target position
        while self.odom_pose == None :
            print ("wait for odom callback")

        target_position = [target_position[0] + self.odom_pose.position.x,
                            target_position[1] + self.odom_pose.position.y]


        # If distance is less than the target, continue moving the robot
        dist = 1000
        cmd = Twist()
        cmd.linear.x = tools.check_speed(self.linear_speed) * direction[0]
        cmd.linear.y = tools.check_speed(self.linear_speed) * direction[1]

        while dist > 0.015 :
            dist = tools.dist2d(target_position, tools.pos2arr(self.odom_pose.position)[:2])
            self.publisher_cmd_vel.publish(cmd)

        cmd.linear.x = 0
        cmd.linear.y = 0
        self.publisher_cmd_vel.publish(cmd)

    def rotate_odom(self, target_rad, debug=False):
        """Command Ridgeback to rotate to certain angle via odometry"""
        command = Twist()
        while self.odom_yaw == None :
            print ("wait for odom callback")
        while True:
            if target_rad-self.odom_yaw > 0:
                command.angular.z = self.angular_speed
            if target_rad-self.odom_yaw < 0:
                command.angular.z = self.angular_speed * (-1)
            self.publisher_cmd_vel.publish(command)

            if debug: print(f'current: {self.odom_yaw}, target: {target_rad}')

            if abs(self.odom_yaw - target_rad) < 0.005:
                break

    def tracker_calibration (self, move=True, debug=True):
        """Calibrate the {vive_world} with physical world"""
        rospy.sleep(1)
        while self.odom_pose == None or self.tracker.pose == None:
            print ("wait for odom and tracker callback")

        # set current position as origin
        if move: self.tracker.origin = tools.pos2arr(self.tracker.pose.position)
        else: self.tracker.origin = np.array([-0.55987096, -0.10827881, 0.83399117])
        if debug: print(f'origin: {self.tracker.origin}')
        rospy.sleep(1)


        # rotate 360 degree in place to calculate the radius
        if move:
            # move 180 degree
            self.rotate_odom(np.pi)
            rospy.sleep(1)
            p1 = tools.pos2arr(self.tracker.pose.position)
            # move back to 0
            self.rotate_odom(0)
            rospy.sleep(1)
            p2 = tools.pos2arr(self.tracker.pose.position)
        else:
            p1 = np.array([-0.6208719, 0.77725536, 0.83583295])
            p2 = np.array([-0.56664193, -0.11963105, 0.83770931])

        self.r = np.linalg.norm(p1-p2)/2.0
        if debug: print(f'p1: {p1}, p2: {p2}')


        # Move forward, set x-axis
        if move:
            self.move_relative([0.5,0])
            rospy.sleep(1)
            x_axis_position = tools.pos2arr(self.tracker.pose.position)
        else: x_axis_position = np.array([-0.58983469, 0.3753503, 0.83545434])
        if debug: print(f'x_axis_position: {x_axis_position}')

        self.tracker.x_axis = x_axis_position - self.tracker.origin
        self.tracker.x_axis = self.tracker.x_axis / np.linalg.norm(self.tracker.x_axis)
        rot_x = tools.get_rotation(self.tracker.x_axis, np.array([1,0,0]))


        # Move left, set y-axis
        if move:
            self.move_relative([0,0.5]) # set axis
            rospy.sleep(1)
            y_axis_position = tools.pos2arr(self.tracker.pose.position)
        else: y_axis_position = np.array([-1.07183957, 0.35897636, 0.83577967])
        if debug: print(f'y_axis_position: {y_axis_position}')

        self.tracker.y_axis = y_axis_position - x_axis_position
        self.tracker.y_axis = self.tracker.y_axis / np.linalg.norm(self.tracker.y_axis)
        transformed_y = np.matmul(rot_x.as_matrix(), self.tracker.y_axis)
        rot_y = tools.get_rotation(transformed_y, np.array([0,1,0]))


        # rotate tracker, set x-axis
        quat = tools.quat2arr(self.tracker.pose.orientation)
        vec = np.matmul(R.from_quat(quat).as_matrix(), np.array([1,0,0]))
        self.tracker.rot_x = tools.get_rotation(vec, self.tracker.x_axis)


        # Transformation from {vive_world} to {real_world}
        rot = rot_y * rot_x
        self.tracker.rot_M = rot.as_matrix()
        # rot = rot * rot_t
        self.tracker.rot_q = rot.as_quat()
        if debug:
            print(f'tracker rotation matrix: {self.tracker.rot_M}')
            print(f'tracker rotation quaternion: {self.tracker.rot_q}')
        rospy.sleep(1)


    def transform_tracker_position(self):
        position = tools.pos2arr(self.tracker.pose.position)
        quaternion = tools.quat2arr(self.tracker.pose.orientation)

        position = position - self.tracker.origin
        position = np.matmul(np.matmul(self.tracker.rot_M, self.tracker.rot_x.as_matrix()), position)
        quaternion = tools.quatmul(tools.quatmul(self.tracker.rot_q, self.tracker.rot_x.as_quat()), quaternion)

        odom_msgs = Odometry()
        odom_msgs.header.stamp = rospy.Time.now()
        odom_msgs.header.frame_id = "vive_world"
        odom_msgs.child_frame_id = "base_link"
        odom_msgs.pose.pose = Pose(Point(position[0], position[1], position[2]),
                                   Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        # send ridgeback radius in twist
        odom_msgs.twist.twist = Twist(Vector3(self.r,0,0), Vector3(0,0,0))
        # publish transformed tracker pose in message
        self.publisher_tracker.publish(odom_msgs)

        # transform wall tracker pose
        position = tools.pos2arr(self.wall.pose.position)
        quaternion = tools.quat2arr(self.wall.pose.orientation)

        position = position - self.tracker.origin
        position = np.matmul(self.tracker.rot_M, position)
        quaternion = tools.quatmul(self.tracker.rot_q, quaternion)
        self.wall.pose.position = Point(position[0], position[1], position[2])
        self.wall.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

        wall_msgs = Odometry()
        wall_msgs.header.stamp = rospy.Time.now()
        wall_msgs.header.frame_id = "vive_world"
        wall_msgs.pose.pose = Pose(Point(position[0], position[1], position[2]),
                                   Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        self.publisher_wall.publish(wall_msgs)

if __name__=='__main__':

    rospy.init_node('ridgeback_tracker')
    rate = rospy.Rate(50)
    calib = Calibration()
    calib.tracker_calibration(move=True, debug=False)

    while not rospy.is_shutdown():
        # publish the transformed tracker position
        calib.transform_tracker_position()
        rate.sleep()
    
