#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PoseArray, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import *
import tools
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from trajectory_script_algorithm import *


class Ridgeback:

    def __init__(self):
        self.iiwa_state = 0
        self.ridgeback_state = 0

        self.publisher_pose = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback/pose', Pose, queue_size=10)
        self.publisher_state = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback/state', Int32, queue_size=10)
        self.publisher_range = rospy.Publisher('/iiwa_ridgeback_communicaiton/drawing_range', Float64MultiArray, queue_size=10)
        self.publisher_traj = rospy.Publisher('/iiwa_ridgeback_communicaiton/trajectory', PoseArray, queue_size=100)
        self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.subscriber_tracker = rospy.Subscriber('/vive/ridgeback', Odometry, self.tracker_callback, queue_size=1)
        self.subscriber_wall = rospy.Subscriber('/vive/wall', Pose, self.wall_callback, queue_size=1)
        self.subscriber_iiwa = rospy.Subscriber("/iiwa_ridgeback_communicaiton/iiwa/state", Int32, self.iiwa_callback, queue_size=1)
        self.subscriber_odom = rospy.Subscriber('/ridgeback_velocity_controller/odom', Odometry, self.odom_callback, queue_size=1)

        self.linear_speed = 0.05
        self.angular_speed = 0.015
        self.r = 0.0

        self.pose = None
        self.ori = None
        self.yaw = None

        #config
        self.wall_file_name = rospy.get_param('/wall_file_name')
        self.wall_pose = None
        self.wall_yaw = None

        # planning result
        self.iiwa_range_list = []
        self.path_x = []
        self.path_y = []
        self.path_angle = []

        self.odom_yaw = None
        self.odom_pose = None

    def iiwa_callback(self, msg):
        self.iiwa_state = msg.data

    def odom_callback(self, msg):
        """Return real robot odometry"""
        self.odom_pose = msg.pose.pose
        qaut = np.array([self.odom_pose.orientation.x,self.odom_pose.orientation.y,self.odom_pose.orientation.z,self.odom_pose.orientation.w])
        self.odom_yaw = R.from_quat(qaut).as_euler('zyx')[0]

    def tracker_callback(self, msg):
        """Return transformed vive tracker value"""
        self.pose = msg.pose.pose
        self.ori = np.array([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w])
        self.yaw = R.from_quat(self.ori).as_euler('zyx')[0]

    def wall_callback(self, msg):
        """Return transformed vive tracker value"""
        self.wall_pose = msg
        wall_ori = msg.orientation
        qaut = np.array([wall_ori.x,wall_ori.y,wall_ori.z,wall_ori.w])
        self.wall_yaw = R.from_quat(qaut).as_euler('zyx')[0]

    def publish_state(self, state):
        self.ridgeback_state = state
        msg = Int32()
        msg.data = state
        self.publisher_state.publish(msg)

    def publish_pose(self):
        while self.pose == None :
            print ("wait for tracker callback")
        p = Pose()
        p.position = Point(self.pose.position.x, self.pose.position.y, 0.0)
        # p.position.x = self.pose.postion.x
        # p.position.y = self.pose.position.y
        p.orientation.x = self.yaw
        self.publisher_pose.publish(p)

    def rotate(self, target_rad, debug=False):
        """Command Ridgeback to rotate to certain angle via tracker"""
        cmd = Twist()
        while self.yaw == None :
            print ("wait for tracker callback")
        while True:
            if target_rad-self.yaw > 0:
                cmd.angular.z = self.angular_speed
            if target_rad-self.yaw < 0:
                cmd.angular.z = self.angular_speed * (-1)
            self.publisher_cmd_vel.publish(cmd)

            if debug: print(f'current: {self.yaw}, target: {self.target_rad}')

            if abs(self.yaw - target_rad) < 0.02:
                break

    def move_relative(self, target_position):
        """Command Ridgeback to move to relative position via odometry"""
        # set direction
        direction = np.array(target_position)
        direction = direction / np.linalg.norm(direction)

        # If distance is less than the target, continue moving the robot
        iter = 0
        cmd = Twist()
        cmd.linear.x = tools.check_speed(self.linear_speed) * direction[0]
        cmd.linear.y = tools.check_speed(self.linear_speed) * direction[1]

        dist = tools.dist2d(np.array(target_position), np.array([0,0]))

        while iter < dist * 100 * 1000 :
            self.publisher_cmd_vel.publish(cmd)
            iter = iter + 1

        cmd.linear.x = 0
        cmd.linear.y = 0
        self.publisher_cmd_vel.publish(cmd)

    def move (self, target_pos=np.array([0,0]), target_yaw=None, debug=False):
        """Move to the given 2D goal pose (position, orientation) in {real_world}"""
        while self.pose == None or self.wall_pose == None:
            print ("wait for tracker callback")

        target_pos = target_pos[:2]
        print(f'move to: {target_pos} in yaw: {target_yaw}')
        dist = 1000
        # If distance is less than the target, continue moving the robot
        while dist > 0.02 :
            current_pos = tools.pos2arr(self.pose.position)[:2]
            # Update direction in run time
            rot_mat = np.array([[math.cos(self.yaw), math.sin(self.yaw)],
                            [-math.sin(self.yaw), math.cos(self.yaw)]])
            direction = target_pos - current_pos
            direction = np.matmul(rot_mat, direction)
            direction = direction / np.linalg.norm(direction)
            if debug: print(direction)

            cmd = Twist()
            if dist < 0.1:
                cmd.linear.x = tools.check_speed(self.linear_speed)/2 * direction[0]
                cmd.linear.y = tools.check_speed(self.linear_speed)/2 * direction[1]
            else:
                cmd.linear.x = tools.check_speed(self.linear_speed) * direction[0]
                cmd.linear.y = tools.check_speed(self.linear_speed) * direction[1]

            if debug: print(f'distance: {dist}, current: {self.pose.position.x}, {self.pose.position.y}')
            dist = tools.dist2d(target_pos, current_pos)
            self.publisher_cmd_vel.publish(cmd)

        cmd.linear.x = 0
        cmd.linear.y = 0
        self.publisher_cmd_vel.publish(cmd)

        if target_yaw is not None:
            self.rotate(target_yaw)

    def read_file(self):
        """Temporary"""
        import rospkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('large_scale_drawing')

        try:
            with open(package_path + '/data/demo/bee_hive_three_traj.txt', 'r') as f:
                for line in f:
                    line = line.split()
                    if line[0].split(':')[0] == 'iiwa_range_list':
                        self.iiwa_range_list = list(map(float,line[1:]))
                    elif line[0].split(':')[0] == 'path_x':
                        self.path_x = list(map(float,line[1:]))
                    elif line[0].split(':')[0] == 'path_y':
                        self.path_y = list(map(float,line[1:]))
                    elif line[0].split(':')[0] == 'path_angle':
                        self.path_angle = list(map(float,line[1:]))
            f.close()
        except IOError:
            print("No demo file exists\n")

    def set_message(self, path_x, path_y, path_angle):
        p = Point()
        q = Quaternion()
        pose = Pose()

        # NOT QUATERNION. orientation x is the theta value of z-rotation in degree
        p.x = path_x
        p.y = path_y
        p.z = 0
        q.x = path_angle

        pose.position = p
        pose.orientation = q

        return pose

    def publish_trajectory(self):
        # self.iiwa_range_list, self.path_x, self.path_y, self.path_angle, = run_algorithm(self.wall_file_name)
        self.read_file()

        message= PoseArray()
        pose_list = []
        for i in range(len(self.path_x)):
            pose = Pose()
            pose = self.set_message (self.path_x[i], self.path_y[i], self.path_angle[i])
            pose_list.append(pose)

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        message.header = h
        message.poses = pose_list
        rate = rospy.Rate(10)

        # drawing range
        range_msg = Float64MultiArray()
        range_msg.data = self.iiwa_range_list

        for _ in range(10):
            # self.publisher_range.publish(range_msg)
            self.publisher_traj.publish(message)
            rate.sleep()

    def move_path(self, path, angles, range_num):
        # self.publish_state(1)
        target_pos = np.array([path[range_num][0], path[range_num][1]])
        target_yaw = angles[range_num]

        # rotate target wrt wall tracker pose
        r = R.from_quat([self.wall_pose.orientation.x,self.wall_pose.orientation.y,self.wall_pose.orientation.z,self.wall_pose.orientation.w])
        target_pos = np.matmul(r.as_matrix(), np.append(target_pos, np.array([0])))
        target_pos = target_pos[:2] + np.array([self.wall_pose.position.x, self.wall_pose.position.y])

        target_yaw = target_yaw * np.pi/180 + self.wall_yaw

        # self.move_relative([-0.2,0.0])
        self.move(np.array(target_pos),target_yaw)

    def follow_trajectory(self, path, angles):
        for i in range(len(angles)):
            self.publish_state(1)
            target_pos = np.array([path[i][0], path[i][1]])
            target_yaw = angles[i]

            # rotate target wrt wall tracker pose
            r = R.from_quat([self.wall_pose.orientation.x,self.wall_pose.orientation.y,self.wall_pose.orientation.z,self.wall_pose.orientation.w])
            target_pos = np.matmul(r.as_matrix(), np.append(target_pos, np.array([0])))
            target_pos = target_pos[:2] + np.array([self.wall_pose.position.x, self.wall_pose.position.y])

            target_yaw = target_yaw * np.pi/180 + self.wall_yaw

            # self.move_relative([-0.2,0.0])
            self.move(np.array(target_pos),target_yaw)

            rospy.loginfo("DONE moving")
            self.iiwa_state = -1
            self.publish_state(0)
            rospy.sleep(2)

            while self.ridgeback_state == 0: # iiwa moving
                if self.iiwa_state != 1: # iiwa waiting
                    rospy.sleep(2)
                    self.publish_pose()
                    # rospy.sleep(2)
                if self.iiwa_state == 2: # change color
                    rospy.sleep(2)
                    # self.move_relative([-0.2,0.0])
                    # input("Press Enter to continue...")
                    # rospy.sleep(2)
                    # self.move(np.array(target_pos),target_yaw)
                    self.publish_pose()
                    self.iiwa_state = 1
                    self.ridgeback_state = 1

        self.publish_pose()
        rospy.sleep(2)

    def save_file(self):
        import rospkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('large_scale_drawing')

        with open(package_path + '/data/demo/' + self.wall_file_name + '_traj.txt', 'w') as f:
            f.write("iiwa_range_list: ")
            for el in self.iiwa_range_list:
                f.write("%f " %el)
            f.write("\npath_x: ")
            for el in self.path_x:
                f.write("%f " %el)
            f.write("\npath_y: ")
            for el in self.path_y:
                f.write("%f " %el)
            f.write("\npath_angle: ")
            for el in self.path_angle:
                f.write("%f " %el)
        f.close()

    def read_file(self):
        import rospkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('large_scale_drawing')

        try:
            with open(package_path + '/data/demo/' + self.wall_file_name + '_traj.txt', 'r') as f:
                for line in f:
                    line = line.split()
                    if line[0].split(':')[0] == 'iiwa_range_list':
                        self.iiwa_range_list = list(map(float,line[1:]))
                    elif line[0].split(':')[0] == 'path_x':
                        self.path_x = list(map(float,line[1:]))
                    elif line[0].split(':')[0] == 'path_y':
                        self.path_y = list(map(float,line[1:]))
                    elif line[0].split(':')[0] == 'path_angle':
                        self.path_angle = list(map(float,line[1:]))
            f.close()
        except IOError:
            print("No demo file exists. Run the algorithm\n")
            self.iiwa_range_list, self.path_x, self.path_y, self.path_angle, = run_algorithm(self.wall_file_name)
            self.save_file()

    def publish_trajectory(self):
        # self.iiwa_range_list, self.path_x, self.path_y, self.path_angle, = run_algorithm(self.wall_file_name)
        self.read_file()

        message= PoseArray()
        pose_list = []
        for i in range(len(self.path_x)):
            pose = Pose()
            pose = self.set_message (self.path_x[i], self.path_y[i], self.path_angle[i])
            pose_list.append(pose)

        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        message.header = h
        message.poses = pose_list
        rate = rospy.Rate(10)

        # drawing range
        range_msg = Float64MultiArray()
        range_msg.data = self.iiwa_range_list

        for _ in range(10):
            self.publisher_range.publish(range_msg)
            self.publisher_traj.publish(message)
            rate.sleep()

        print()
        print('X',self.path_x)
        print('Y',self.path_y)
        print('iiwa',self.iiwa_range_list)
        # print(self.path_angle)


if __name__=='__main__':

    rospy.init_node('cmd_ridgeback')
    Rid = Ridgeback()

    Rid.read_file()
    Rid.publish_trajectory()
    i = 3

    try:
        while Rid.wall_pose == None:
            print ("wait for tracker callback")
        # Rid.follow_trajectory(list(zip(Rid.path_x, Rid.path_y)), Rid.path_angle)
        Rid.publish_pose()
        # Rid.move_path(list(zip(Rid.path_x, Rid.path_y)), Rid.path_angle, i)
        # Rid.move_relative([-1.0,-1.0])
    except rospy.ROSIntecrruptException:
        pass
