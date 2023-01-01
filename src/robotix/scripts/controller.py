#!/usr/bin/env python3

import rospy
import sys
from select import select
from geometry_msgs.msg import Twist
import termios
import tty

move_topic = "/robot/robotnik_base_control/cmd_vel"

msg = """
start moving the SUMMIT_XL robot with
w => forward, s => backword
a => left, d => right
"""


def getKey(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == "__main__":
    rospy.init_node("move_robot")
    rospy.loginfo("Starting move_robot node")
    pub = rospy.Publisher(move_topic, Twist, queue_size=10)

    rate = rospy.Rate(10)
    msg = Twist()
    l_acc = 2
    a_acc = 2

    settings = saveTerminalSettings()

    while not rospy.is_shutdown():
        try:
            key = getKey(settings, 0.1)
            if key == "w" and msg.linear.x <= l_acc:
                msg.linear.x += l_acc
                pub.publish(msg)
            elif key == "s" and msg.linear.x >= -l_acc:
                msg.linear.x -= l_acc
                pub.publish(msg)
            elif key == "a" and msg.angular.z <= a_acc:
                msg.angular.z += a_acc
                pub.publish(msg)
            elif key == "d" and msg.angular.z >= -a_acc:
                msg.angular.z -= a_acc
                pub.publish(msg)
            elif msg.linear != 0 or msg.angular != 0:
                msg.linear.x = 0
                msg.angular.z = 0
                pub.publish(msg)

        except Exception as e:
            print(e)

        rate.sleep()

    restoreTerminalSettings(settings)
