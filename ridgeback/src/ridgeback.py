#! /usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import math
import numpy as np
from geometry_msgs.msg import PoseArray, Point, Quaternion, Pose, Twist
from std_msgs.msg import *
from trajectory_script_algorithm import *
'''
rostopic pub /iiwa_ridgeback_communicaiton/iiwa std_msgs/String -- \'0\'
'''
class Ridgeback:

    def __init__(self):

        #init
        self.iiwa_state = "0" # ridgeback moves first to the first drawing pose

        self.publisher_pose = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback/pose', Pose, queue_size=10)
        self.publisher_state = rospy.Publisher('/iiwa_ridgeback_communicaiton/ridgeback/state', String, queue_size=10)
        self.publisher_range = rospy.Publisher('/iiwa_ridgeback_communicaiton/drawing_range', Float64MultiArray, queue_size=10)
        self.publisher_traj = rospy.Publisher('/iiwa_ridgeback_communicaiton/trajectory', PoseArray, queue_size=100)
        self.publisher_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subscriber_odom = rospy.Subscriber("/odometry/filtered", Odometry, self.callback_odom)
        self.subscriber_iiwa = rospy.Subscriber("/iiwa_ridgeback_communicaiton/iiwa/state", String, self.callback_iiwa)
        self.ridgeback_state = 0
        self.linear_speed = 0.05
        self.kp = 0.01 # angular speed
        self.reached = False
        self.i = 0

        #pose
        self.position_x = None
        self.position_y = None
        self.orientation_q = None

        #config
        self.wall_file_name = rospy.get_param('/wall_file_name')
        self.wall_pose = rospy.get_param('/wall_pose')
        #result
        self.iiwa_range_list = []
        self.path_x = []
        self.path_y = []
        self.path_angle = []

    def euler_from_quaternion(self, orientation_list):

        [x, y, z, w] = orientation_list
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z 

    def callback_iiwa(self, msg):
        self.iiwa_state = msg.data
        # print('iiwa state is:',self.iiwa_state)

    # TODO: use pure quaternion
    def callback_odom(self, msg):
        # get position
        pose_position = msg.pose.pose.position
        self.position_x = pose_position.x
        self.position_y = pose_position.y

        # get orientation
        self.orientation_q = msg.pose.pose.orientation
        orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]

        (roll, pitch, self.yaw) = self.euler_from_quaternion(orientation_list)
        if self.yaw < 0:
            angle = -self.yaw*180/math.pi
        else:
            angle = 360-self.yaw*180/math.pi

        self.rad = math.radians(angle)

    def fixed_goal(self, goal_x, goal_y):
        while self.position_x == None:
            print("wait for odom callback ..")

        print('Executing ridgeback')
        cmd = Twist()
        print(f'MOVING to the goal #{goal_x}, #{goal_y}')

        while not self.reached:
            x_move = goal_x-self.position_x
            y_move = goal_y-self.position_y
            self.i +=1
            if self.i % 300000 == 0:
                print("distance: ", dist)
            goal = np.array([[x_move],[y_move]])

            rotation_matrix = np.array([[math.cos(self.rad), -math.sin(self.rad)], 
                                        [math.sin(self.rad), math.cos(self.rad)]])

            r_goal = np.matmul(rotation_matrix, goal)
            r_goal = r_goal / np.linalg.norm(r_goal) # needs to be unit vector
            dist = self.calculate_distance(goal)
            
            if not self.reached:
                if dist > 0.01:
                    cmd.linear.x = self.linear_speed * r_goal[0][0]
                    cmd.linear.y = self.linear_speed * r_goal[1][0]

                if dist < 0.01:
                    cmd.linear.x = 0
                    cmd.linear.y = 0
                    self.reached = True
                    print('DONE')
                    
            self.publisher_cmd_vel.publish(cmd)
        self.reached = False
        return True

    def calculate_distance(self, r_goal):
        return math.sqrt(r_goal[0]**2+ r_goal[1]**2)

    def follow_trajectory(self, path, angles):
        for i in range(len(angles)):
            # self.fixed_rotate(0)
            self.publish_state(1) # ridgeback moving
            result = self.fixed_goal(path[i][0] + self.wall_pose[0], path[i][1] + self.wall_pose[1])

            if result:
                rospy.loginfo("Goal execution done!")
                rospy.loginfo("Rotating to calculated angle >>")
                self.fixed_rotate(angles[i])
                rospy.loginfo("DONE rotating >>")
                self.iiwa_state = "-1"
                self.publish_state(0) # ridgeback done

            else:
                raise('unknown error')

            while self.iiwa_state != "0": # iiwa moving
                if self.iiwa_state != "1": # iiwa waiting
                    rospy.sleep(2)
                    self.publish_pose()
                    rospy.sleep(2)

        self.publish_pose()
        rospy.sleep(2)

    def publish_state(self, number):
        s = String()
        s.data = str(number)
        self.publisher_state.publish(s)

    def publish_pose(self):
        P = Pose()
        P.position.x = self.position_x
        P.position.y = self.position_y
        P.orientation.x = self.yaw
        # P.orientaiton = self.orientation_q
        self.publisher_pose.publish(P)

    def fixed_rotate(self, target_angle):
        command =Twist()

        if 0 <= target_angle <= 180:
            real_target = target_angle
        elif 180 < target_angle <= 360:
            real_target = 180 - target_angle
        elif -360 <= target_angle <= -180:
            real_target = target_angle + 360
        else:
            real_target = target_angle

        while True:
            target_rad = real_target*math.pi/180 # radian
            if target_rad-self.yaw > 0:
                command.angular.z = self.kp
            if target_rad-self.yaw < 0:
                command.angular.z = self.kp * (-1)
            self.publisher_cmd_vel.publish(command)

            if abs(self.yaw*180/math.pi - real_target) < 0.1:
                break

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

if __name__ == '__main__':

    rospy.init_node('RIDGEBACK', anonymous=True)

    Rid = Ridgeback()
    rospy.sleep(2)
    Rid.publish_trajectory()
    print('publish trajectory done')
    # Rid.fixed_rotate(0)  # temp
    print('rotated to 0 degrees for the first time')
    path = list(zip(Rid.path_x, Rid.path_y))
    # Rid.follow_trajectory(path, Rid.path_angle)
    # i = 0
    # i = 1
    i = 2
    # i = 3
    # i = 4
    result = Rid.fixed_goal(path[i][0] + Rid.wall_pose[0], path[i][1] + Rid.wall_pose[1])
    result = Rid.fixed_rotate(Rid.path_angle[i])

