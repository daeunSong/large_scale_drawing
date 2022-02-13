#!/usr/bin/env python3

# https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-3d/
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time

# ============= VARIABLES ============== #
graph_limit = 3

class Tracker:

    def __init__(self):
        self.vive_pose0 = []
        self.vive_pose1 = []
        self.i = 0
        self.transform_vector = []
        self.R_M = None
        self.vive_orientation = []
        
    def get_RM(self, vector):
    
        angle = self.angle_between(vector, np.array([1,0,0]))
        R_My, now = self.rotate_y(vector, angle)
        angle = self.angle_between(now, np.array([0,1,0]))
        R_Mz, now = self.rotate_z(now, angle)
        angle = self.angle_between(now, np.array([0,0,1]))
        R_Mx, now = self.rotate_x(now, angle)

        # angle = angle_between(vector, np.array([0,0,1]))
        # R_Mx, now = rotate_y(vector, angle)
        # angle = angle_between(now, np.array([1,0,0]))
        # R_My, now = rotate_z(now, angle)
        # angle = angle_between(now, np.array([0,1,0]))
        # R_Mz, now = rotate_x(now, angle)

        self.R_M = np.dot(R_Mz, R_My)
        self.R_M = np.dot(R_Mx, self.R_M)
        print('R_M made')

    def transform_RM(self, vector):
        # print(self.R_M)
        # print(vector)
        return np.dot(self.R_M, vector)

    def angle_between(self, v1, v2):
        # angle between two vectors in radians
        unit_vector_1 = v1 / np.linalg.norm(v1)
        unit_vector_2 = v2 / np.linalg.norm(v2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        return np.arccos(dot_product)

    def rotate_x(self, vector, angle):

        theta_rx = angle
        sin_rx, cos_rx = np.sin(theta_rx), np.cos(theta_rx)

        # get the rotation matrix on x axis
        R_Mx = np.array([[1,      0,       0],
                        [0, cos_rx, -sin_rx],
                        [0, sin_rx,  cos_rx]])

        after = np.dot(R_Mx,vector)
        # plot_vector(after, 'lightcoral')
        return R_Mx, after

    def rotate_y(self, vector, angle):

        theta_rx = angle
        sin_ry, cos_ry = np.sin(theta_rx), np.cos(theta_rx)

        # get the rotation matrix on y axis
        R_My = np.array([[cos_ry, 0, -sin_ry],
                            [     0, 1,       0],
                            [sin_ry, 0,  cos_ry]])

        after = np.dot(R_My,vector)
        # plot_vector(after, 'cyan')
        return R_My, after

    def rotate_z(self, vector, angle):

        theta_rx = angle
        sin_rz, cos_rz = np.sin(theta_rx), np.cos(theta_rx)

        # get the rotation matrix on z axis
        R_Mz = np.array([[cos_rz, sin_rz, 0],
                            [-sin_rz,  cos_rz, 0],
                            [     0,       0, 1]])

        after = np.dot(R_Mz,vector)
        # plot_vector(after, 'lime')
        return R_Mz, after


class Ridgeback:

    def __init__(self):
        self.tracker_name = '515D3307'
        # self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_pose = rospy.Publisher('/vive_pose/filtered', Odometry, queue_size=1)
        self.subscriber_vive = rospy.Subscriber('/vive/LHR_'+self.tracker_name+'_pose', PoseWithCovarianceStamped, self.get_current_position)
        
        self.linear_speed = 0.1
        self.reached = False
        self.i = 0
        self.linear_x, self.linear_y, self.linear_z = [],[],[]
        self.tracker = Tracker()
        self.linear_vx, self.linear_vy, self.linear_vz = [],[],[]
        self.timestamps={'step0':10, 'step1':50, 'step2':100}

    def get_current_position(self,msg):

        pose_position = msg.pose.pose.position
        position_x = pose_position.x
        position_y = pose_position.y
        position_z = pose_position.z

        pose_orientation = msg.pose.pose.orientation
        orientation_x = pose_orientation.x
        orientation_y = pose_orientation.y
        orientation_z = pose_orientation.z
        self.tracker.vive_orientation = [orientation_x, orientation_y, orientation_z]

        self.i += 1

        if self.i<= self.timestamps['step0']:
            self.linear_x.append(round(position_x,3))
            self.linear_y.append(round(position_y,3))
            self.linear_z.append(round(position_z,3))

        self.tracker.i+=1

        if self.tracker.i<=self.timestamps['step0']:
            print('init pose')
            self.tracker.vive_pose0 = [position_x, position_y, position_z]
            # self.tracker.vive_pose1 = [position_x, position_y, position_z]

        # else:
        self.tracker.vive_pose1 = [position_x, position_y, position_z]
        
        if self.timestamps['step0'] == self.tracker.i:
            # self.move_relative(0, -0.4)
            rospy.sleep(10)

        if self.timestamps['step1']<self.tracker.i<self.timestamps['step2']:
            print('move right')
            moved_vector = np.array(self.tracker.vive_pose1)- np.array(self.tracker.vive_pose0)

            self.tracker.get_RM(moved_vector)
            
        if self.tracker.i == self.timestamps['step2']:
            print('starting to print real value')
            print(self.tracker.R_M)

        if self.tracker.i > self.timestamps['step2']:
            world_frame = self.tracker.transform_RM(self.tracker.vive_pose1)
            world_frame_or = self.tracker.transform_RM(self.tracker.vive_orientation)
            self.linear_vz.append(world_frame[0])
            self.linear_vy.append(world_frame[1])
            self.linear_vx.append(world_frame[2])

            odom_msgs = Odometry()
            odom_msgs.pose.pose.position.x = world_frame[2]
            odom_msgs.pose.pose.position.y = world_frame[1]
            odom_msgs.pose.pose.position.z = world_frame[0]

            odom_msgs.pose.pose.orientation.x = world_frame_or[2]
            odom_msgs.pose.pose.orientation.y = world_frame_or[1]
            odom_msgs.pose.pose.orientation.z = world_frame_or[0]

            self.publisher_pose.publish(odom_msgs)

            print(f'x: {round(world_frame[2],3)}, y:{round(world_frame[1],3)}')

    def check_speed(self, speed):
        if speed >= 0.03:
            return 0.03
        else:
            return speed

    def move_relative(self, x, y, duration=5):
        print("moving to :", x, y)

        cmd = Twist()
        cmd.linear.x = self.check_speed(x/duration)
        cmd.linear.y = self.check_speed(y/duration)
        cmd.linear.z = 0

        rospy.sleep(1)

        seconds = time.time()
        while time.time() - seconds < duration:
            self.publisher_cmd_vel.publish(cmd)

    def animate(self, i):
        ax.clear()
        ax.set_xlim(-graph_limit, graph_limit)
        ax.set_ylim(-graph_limit, graph_limit)
        ax.set_zlim(-graph_limit, graph_limit)

        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        ax.set_title('Tracker Pose')

        ax.plot3D(self.linear_x, self.linear_y, self.linear_z, color='r')
        ax.plot3D(self.linear_vx, self.linear_vy, self.linear_vz, color='b')

    def start(self):
        ani = animation.FuncAnimation(fig, self.animate, interval=10)
        plt.show()

# def plot_vector(vector, color):
#     ax.plot3D([0,vector[0]], [0,vector[1]], [0,vector[2]], color)


if __name__=='__main__':

    fig = plt.figure()
    ax = fig.gca(projection='3d')

    rospy.init_node('print_tracker_pose')

    Rid = Ridgeback()
    Rid.start()
    
