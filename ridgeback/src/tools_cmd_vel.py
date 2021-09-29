import rospy
from geometry_msgs.msg import Twist
import time
import math
# angle related


def turn_right(z, sec):
    print("right")
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = -z
    rospy.sleep(1)
    seconds = time.time()
    while time.time() - seconds < sec:
        publisher.publish(cmd)


def turn_left(z, sec):
    print("left")
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = z
    rospy.sleep(1)
    seconds = time.time()
    while time.time() - seconds < sec:
        publisher.publish(cmd)


# movement related


def move_forward(speed, duration):
    print("forward")
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.linear.x = speed
    cmd.linear.y = 0
    cmd.linear.z = 0
    rospy.sleep(1)
    seconds = time.time()
    while time.time() - seconds < duration:
        publisher.publish(cmd)


def move_backward(speed, duration):
    print("backward")
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    cmd = Twist()
    cmd.linear.x = -speed
    cmd.linear.y = 0
    cmd.linear.z = 0
    rospy.sleep(1)
    seconds = time.time()
    while time.time() - seconds < duration:
        publisher.publish(cmd)


def move_relative(x, y, duration=5):
    print("moving to :", x, y)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    cmd = Twist()
    cmd.linear.x = x/duration
    cmd.linear.y = y/duration
    cmd.linear.z = 0

    rospy.sleep(1)

    seconds = time.time()
    while time.time() - seconds < duration:
        publisher.publish(cmd)


def move_relative_rotate(x, y, angle=5, duration=5):
    print("moving to :", x, y)
    publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    cmd = Twist()
    cmd.linear.x = -y/duration
    cmd.linear.y = -x/duration
    cmd.linear.z = 0

    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = math.radians(angle)/duration

    rospy.sleep(1)

    seconds = time.time()
    while time.time() - seconds < duration:
        publisher.publish(cmd)
