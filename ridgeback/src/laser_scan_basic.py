#! /usr/bin/env python3
from sensor_msgs.msg import LaserScan
import rospy
import tools_etc


def callback_laser(msg):

    # 1. check the range of laser scanner and change the code accordingly
    max_range = len(msg.ranges)
    # print (len(msg.ranges))
    # gazebo simulator : 0-720

    # 2. to check distance
    # values at 0 degree
    # print ("0:{:.4}".format(msg.ranges[0]))
    # # values at 90 degree
    # print ("90:{:.4}".format(msg.ranges[max_range/2]))
    # # values at 180 degree
    # print ("180:{:.4}".format(msg.ranges[max_range-1]))

    # 3. check 60, 90, 120 distance
    angle_60 = tools_etc.average(msg.ranges, round(max_range/3))
    angle_90 = tools_etc.average(msg.ranges, round(max_range/2))
    angle_120 = tools_etc.average(msg.ranges, round(max_range*2/3))

    print("{:.4} {:.4} {:.4}".format(angle_60, angle_90, angle_120))


if __name__ == "__main__":
    rospy.init_node('scan_values')
    sub = rospy.Subscriber('/front/scan', LaserScan, callback_laser)
    rospy.spin()

