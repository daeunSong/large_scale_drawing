#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import sys
import tf
import numpy as np

class RealWorld():
    def __init__(self):
        self.translation = None
        self.rotation = None
        self.iiwa_trans = None
        self.iiwa_rot = None
        self.wall_trans = None
        self.wall_rot = None
        self.publisher_pose = rospy.Publisher('/vive/ridgeback', Odometry, queue_size=1)
        self.publisher_iiwa = rospy.Publisher('/vive/iiwa', Pose, queue_size=1)
        self.publisher_wall = rospy.Publisher('/vive/wall', Pose, queue_size=1)
        self.subscriber_pose = rospy.Subscriber('/vive/tracker', Odometry, self.call_back)
        self.subscriber_wall = rospy.Subscriber('/vive/wall_tracker', Odometry, self.wall_call_back)
        self.frame_generated = False
        self.broadcaster = tf.TransformBroadcaster()

    def call_back (self, msg):
        pose_ = msg.pose.pose
        self.translation = [pose_.position.x,pose_.position.y,pose_.position.z]
        self.rotation = [pose_.orientation.x,pose_.orientation.y,pose_.orientation.z,pose_.orientation.w]
        if (self.translation is not None and self.rotation is not None):
            self.broadcaster.sendTransform(self.translation,
                        self.rotation,
                        rospy.Time.now(),
                        'vive_tracker',
                        'vive_world')
            self.broadcaster.sendTransform([0.45,0,0],
                        [0,0,0,1],
                        rospy.Time.now(),
                        'vive_ridgeback',
                        'vive_tracker')
            self.broadcaster.sendTransform([0.125,0,0],
                        [0,0,0,1],
                        rospy.Time.now(),
                        'vive_iiwa',
                        'vive_ridgeback')

    def wall_call_back (self, msg):
        pose_ = msg.pose.pose
        self.translation = [pose_.position.x,pose_.position.y,pose_.position.z]
        self.rotation = [pose_.orientation.x,pose_.orientation.y,pose_.orientation.z,pose_.orientation.w]
        if (self.translation is not None and self.rotation is not None):
            self.broadcaster.sendTransform(self.translation,
                        self.rotation,
                        rospy.Time.now(),
                        'vive_temp',
                        'vive_world')
            self.broadcaster.sendTransform([-0.25,0.03,0],
                        [0,0,0,1],
                        rospy.Time.now(),
                        'vive_wall',
                        'vive_temp') #[-0.32,0.11,0]

if __name__ == '__main__':
    rospy.init_node('vive_tf_publisher')
    listener = tf.TransformListener()
    rate = rospy.Rate(50)

    world = RealWorld()

    # covariance from the vive tracker
    P = np.mat([[1e-6, 0, 0], [0, 1e-6, 0], [0, 0, 1e-3]])
    p_cov = np.zeros((6, 6))
    # position covariance
    p_cov[0:2,0:2] = P[0:2,0:2]
    # orientation covariance for Yaw
    # x and Yaw
    p_cov[5,0] = p_cov[0,5] = P[2,0]
    # y and Yaw
    p_cov[5,1] = p_cov[1,5] = P[2,1]
    # Yaw and Yaw
    p_cov[5,5] = P[2,2]

    p_cov[0,:] = [0.0000349162103240595,  -0.0000018202960310455,  -0.0000339898160507969,  -0.0000081126791170800,   0.0000001353045808767,   0.0000032202291901186]
    p_cov[1,:] = [-0.0000018202960310455,   0.0000011910722363973,   0.0000020423436706964,   0.0000010961526869235,  -0.0000000333091396801,  -0.0000001408541892558]
    p_cov[2,:] = [-0.0000339898160507969,   0.0000020423436706964,   0.0000341312090595451,   0.0000060715616751347,  -0.0000000237628610568,  -0.0000029217229365340]
    p_cov[3,:] = [-0.0000081126791170800,   0.0000010961526869235,   0.0000060715616751347,   0.0000165832615351042,  -0.0000004759697840205,  -0.0000024486872043021]
    p_cov[4,:] = [0.0000001353045808767,  -0.0000000333091396801,  -0.0000000237628610568,  -0.0000004759697840205,   0.0000003366392930324,  -0.0000000030521109214]
    p_cov[5,:] = [0.0000032202291901186,  -0.0000001408541892558,  -0.0000029217229365340,  -0.0000024486872043021,  -0.0000000030521109214,   0.0000007445433570531]
    # rospy.loginfo(p_cov)

    while not rospy.is_shutdown():
        try:
            (world.translation, world.rotation) = listener.lookupTransform('vive_world', 'vive_ridgeback', rospy.Time(0))
            (world.iiwa_trans, world.iiwa_rot) = listener.lookupTransform('vive_world', 'vive_iiwa', rospy.Time(0))
            (world.wall_trans, world.wall_rot) = listener.lookupTransform('vive_world', 'vive_wall', rospy.Time(0))
            if world.frame_generated is False:
                world.frame_generated = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            sys.stdout.write('.')
            if (world.translation is not None and world.rotation is not None):
                world.broadcaster.sendTransform(world.translation,
                                world.rotation,
                                rospy.Time.now(),
                                "vive_ridgeback",
                                "vive_world")
            rate.sleep()

        if world.frame_generated:
            # ridgeback pose msg
            odom_msgs = Odometry()
            odom_msgs.header.stamp = rospy.Time.now()
            odom_msgs.header.frame_id = "vive_world"
            odom_msgs.child_frame_id = "base_link"
            odom_msgs.pose.pose = Pose(Point(world.translation[0], world.translation[1], world.translation[2]),
                                       Quaternion(world.rotation[0], world.rotation[1], world.rotation[2], world.rotation[3]))
            odom_msgs.pose.covariance = tuple(p_cov.ravel().tolist())
            odom_msgs.twist.covariance = tuple(p_cov.ravel().tolist())

            world.publisher_pose.publish(odom_msgs)

            # iiwa pose msg
            pose_msgs = Pose(Point(world.iiwa_trans[0], world.iiwa_trans[1], world.iiwa_trans[2]),
                                       Quaternion(world.iiwa_rot[0], world.iiwa_rot[1], world.iiwa_rot[2], world.iiwa_rot[3]))
            world.publisher_iiwa.publish(pose_msgs)

            # wall pose msg
            wall_msgs = Pose(Point(world.wall_trans[0], world.wall_trans[1], world.wall_trans[2]),
                                       Quaternion(world.wall_rot[0], world.wall_rot[1], world.wall_rot[2], world.wall_rot[3]))
            world.publisher_wall.publish(wall_msgs)
