#! /usr/bin/env python3
from sensor_msgs.msg import LaserScan
import rospy
from geometry_msgs.msg import Twist
from tools import tools_cmd_vel
from nav_msgs.msg import Odometry
import math

init_pose = []
final_pose = []
i = 0
x = 1
y = 1
z = 1

def callback(msg):
    global i, final_pose, init_pose,x, y, z
    
    pose = msg.pose.pose.position
    if i < 5:
        # print(f'init pose: {round(pose.x, 3)}, {round(pose.y, 3)}')
        init_pose = [round(pose.x, 3), round(pose.y, 3)]

    i+=1
    # if i % 80 == 0:
        # print(f'pose: {round(pose.x, 3)}, {round(pose.y, 3)}')
    final_pose = [pose.x, pose.y]
    x = pose.x
    y = pose.y
    z = pose.z

angle_90 = 0
angle_60 = 0
angle_120 = 00

def callback_laser(msg):
    global angle_120, angle_60, angle_90

    # for real ridgeback
    # angle_60 = (msg.ranges[490]+msg.ranges[500]+msg.ranges[510])/3
    # angle_90 = (msg.ranges[530]+msg.ranges[540]+msg.ranges[550])/3
    # angle_120 = (msg.ranges[570]+msg.ranges[580]+msg.ranges[590])/3

    # for simulation
    angle_60 = (msg.ranges[230]+msg.ranges[240]+msg.ranges[250])/3
    angle_90 = (msg.ranges[350]+msg.ranges[360]+msg.ranges[370])/3
    angle_120 = (msg.ranges[470]+msg.ranges[480]+msg.ranges[490])/3



def right_angle():
    thresh = 0.008
    rospy.sleep(1)
    while True:
        if angle_120 - angle_60 > thresh:
            tools_cmd_vel.turn_right(0.05, 0.1)
        elif angle_120 - angle_60 < -thresh:
            tools_cmd_vel.turn_left(0.05, 0.1)
        else:
            print(angle_60, angle_120, angle_120-angle_60)
            print("done?")
            break


def go_to_goal(x_goal, y_goal):

    velocity_message = Twist()
    cmd_vel_topic = '/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
    print('start moving to ', x_goal, y_goal)

    i = 0
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

        if (distance < 0.01):
            print('done')
            return True
    return False

def get_goal( x_var, y_var, distance):
    var = (distance**2 / (x_var**2+y_var**2))**0.5
    goal = [final_pose[0] + var*x_var,final_pose[1] + var*y_var]
    return goal

if __name__ == "__main__":
    rospy.init_node('scan_values')
    sub = rospy.Subscriber('/front/scan', LaserScan,callback_laser)
    right_angle()

    odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, callback)

    # goal x,y : the final goal
    x_goal = 0
    y_goal = -1
    try:
        tools_cmd_vel.move_relative(x_goal, y_goal, duration=10)
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occured.")
        
    print(f'init pose: {init_pose}\nfinal pose:{final_pose}')
    
    variation=[final_pose[0] - init_pose[0], final_pose[1] - init_pose[1]]
    print(f'variation: {variation[0]}, {variation[1]}')
    try:
        rospy.sleep(1)
        goal = get_goal(variation[0], variation[1], 1)
        print(goal)
        start = final_pose
        result = go_to_goal(goal[0], goal[1])
        end = final_pose
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Error occurred")
    distance = ((((start[0] - end[0])**2) + ((start[1]-end[1])**2) )**0.5)
    print(distance)