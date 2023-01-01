#!/usr/bin/env python3

import rospy
import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from robotix.msg import LaserOdom

lasers_topic = "/scan_multi"
odom_topic = "/robot/robotnik_base_control/odom"
results_topic = "/sensors_incorporating"


def callback(lazer: LaserScan, ode: Odometry):
    msg = LaserOdom()
    msg.LaserSensor = lazer
    msg.OdometrySensor = ode
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("sensor_incorporating")
    rospy.loginfo("Sensor Incorporating Node started")

    pub = rospy.Publisher(results_topic, LaserOdom, queue_size=10)
    Laser = message_filters.Subscriber(lasers_topic, LaserScan)
    Ode = message_filters.Subscriber(odom_topic, Odometry)

    ts = message_filters.TimeSynchronizer([Laser, Ode], 10)
    ts.registerCallback(callback)
    rospy.spin()
