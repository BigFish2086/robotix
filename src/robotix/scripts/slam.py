#!/usr/bin/env python3

import rospy
import numpy as np
from skimage.draw import line
from robotix.msg import LaserOdom
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped 

# knowing the map which got published on map_topic
#  which is basically an OccupancyGrid message
# and the laser_odom which got published on laser_odom_topic
#  which is a LaserOdom message (wich combines front, rear and odometry data)

##################################################################################
# topics to get input from
move_topic = "/tf"
laser_odom_topic = "/sensors_incorporating"

# topics to publish results at
map_topic = "/mapping"

# previuse pose configuration
prev_t = 0
prev_robot_x = 0
prev_robot_y = 0
prev_robot_theta = 0

map = OccupancyGrid()
laser_odom = LaserOdom()
move = TransformStamped()
pose = PoseStamped()

hits = np.ndarray
misses = np.ndarray
map_data = np.ndarray


##################################################################################
# just to initilaize the map
def init_map():
    global map, hits, misses, map_data
    map.header.frame_id = "robot_map"

    # initialize map, cells
    map.info.width = 500
    map.info.height = 500
    map.info.resolution = 0.2  # m/cell (1m/50cells)

    # map orientation
    map.info.origin.orientation.x = 0
    map.info.origin.orientation.y = 0
    map.info.origin.orientation.z = 0
    map.info.origin.orientation.w = 1

    # map position
    map.info.origin.position.z = 0
    map.info.origin.position.x = 0 - map.info.resolution * map.info.width / 2
    map.info.origin.position.y = 0 - map.info.resolution * map.info.height / 2

    map_shape = (map.info.width, map.info.height)
    hits = np.full(map_shape, 1)
    misses = np.full(map_shape, 1)
    map_data = np.full(map_shape, -1)


##################################################################################
# kalman filter
def get_observed_pose():
    # get the robot pose from the move topic
    robot_x = move.transform.translation.x
    robot_y = move.transform.translation.y
    robot_theta = move.transform.rotation.z
    return robot_x, robot_y, robot_theta


def get_predicted_pose():
    twist = laser_odom.OdometrySensor.twist.twist
    # get the robot velocity and time
    vx = twist.linear.x
    vy = twist.linear.y
    vtheta = twist.angular.z
    dt = laser_odom.OdometrySensor.header.stamp.to_sec() - prev_t
    avg_theta = (prev_robot_theta + vtheta * dt)
    # calculate the new pose
    robot_x = prev_robot_x + vx * dt * np.cos(avg_theta) - vy * dt * np.sin(avg_theta)
    robot_y = prev_robot_y + vx * dt * np.sin(avg_theta) + vy * dt * np.cos(avg_theta)
    robot_theta = prev_robot_theta + vtheta * dt
    return robot_x, robot_y, robot_theta


def get_pos_kf():
    global prev_t, prev_robot_x, prev_robot_y, prev_robot_theta
    # get the observed and predicted pose
    observed_pose = get_observed_pose()
    predicted_pose = get_predicted_pose()
    # get the covariance matrix of the laser_odom readings
    cov_x = laser_odom.OdometrySensor.pose.covariance[0]
    cov_y = laser_odom.OdometrySensor.pose.covariance[7]
    cov_theta = laser_odom.OdometrySensor.pose.covariance[35]
    # calculate the Kalman gain
    # kx = cov_x / (cov_x + cov_y + cov_theta)
    # ky = cov_y / (cov_x + cov_y + cov_theta)
    # ktheta = cov_theta / (cov_x + cov_y + cov_theta)
    kx = 0.1
    ky = 0.2
    ktheta = 0.1
    # calculate the new pose using the Kalman gain
    robot_new_x = kx * observed_pose[0] + (1 - kx) * predicted_pose[0]
    robot_new_y = ky * observed_pose[1] + (1 - ky) * predicted_pose[1]
    robot_new_theta = ktheta * observed_pose[2] + (1 - ktheta) * predicted_pose[2]
    # update the previuse pose
    prev_t = laser_odom.OdometrySensor.header.stamp.to_sec()
    prev_robot_x = robot_new_x
    prev_robot_y = robot_new_y
    prev_robot_theta = robot_new_theta
    # return the new pose
    return robot_new_x, robot_new_y, robot_new_theta


##################################################################################
# related to drawing the map
# this function should draw the map, using the laser_odom data
# with the robot pose calculated using the Kalman filter
def euler_angles(x, y, z, w):
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([x, y, z, w])
    return roll, pitch, yaw


def get_line_start(world):
    global map
    start_x, start_y, yaw = get_pos_kf()
    # publish on topic slam_loc PoseStamped
    pose.pose.position.x = start_x
    pose.pose.position.y = start_y
    pose.pose.orientation.z = yaw
    posPub.publish(pose)

    # start_x = world.OdometrySensor.pose.pose.position.x
    # start_y = world.OdometrySensor.pose.pose.position.y
    # start_i = int(round((start_x) / map.info.resolution) + map.info.width / 2)
    # start_j = int(round((start_y) / map.info.resolution) + map.info.height / 2)
    # yaw = euler_angles(
    #     world.OdometrySensor.pose.pose.orientation.x,
    #     world.OdometrySensor.pose.pose.orientation.y,
    #     world.OdometrySensor.pose.pose.orientation.z,
    #     world.OdometrySensor.pose.pose.orientation.w,
    # )[2]
    start_i = int(round((start_x) / map.info.resolution) + map.info.width / 2)
    start_j = int(round((start_y) / map.info.resolution) + map.info.height / 2)
    return start_x, start_y, start_i, start_j, yaw


def get_line_end(start_x, start_y, cur_reading, cur_angle):
    global map
    end_x = start_x + cur_reading * np.cos(cur_angle)
    end_y = start_y + cur_reading * np.sin(cur_angle)
    end_i = int(round((end_x) / map.info.resolution) + map.info.width / 2)
    end_j = int(round((end_y) / map.info.resolution) + map.info.height / 2)
    return end_i, end_j


def draw_map(msg):
    global map, hits, misses, map_data
    rospy.loginfo("Start publishing")
    start_x, start_y, start_i, start_j, yaw = get_line_start(msg)
    readings = msg.LaserSensor.ranges
    angle_rad = msg.LaserSensor.angle_min
    for i in range(len(readings)):
        if readings[i] < msg.LaserSensor.range_min or readings[i] > msg.LaserSensor.range_max:
            angle_rad += msg.LaserSensor.angle_increment
            continue
        cur_angle = angle_rad + yaw
        end_i, end_j = get_line_end(start_x, start_y, readings[i], cur_angle)
        beam_y, beam_x = line(r0=start_j, c0=start_i, r1=end_j, c1=end_i)
        for j in range(len(beam_x) - 1):
            if beam_x[j] >= map.info.width or beam_y[j] >= map.info.height:
                continue
            misses[beam_y[j], beam_x[j]] += 1
        if beam_x[-1] >= map.info.width or beam_y[-1] >= map.info.height:
            angle_rad += msg.LaserSensor.angle_increment
            continue
        if readings[i] != msg.LaserSensor.range_max:
            hits[beam_y[-1], beam_x[-1]] += 1
        else:
            misses[beam_y[-1], beam_x[-1]] += 1
        angle_rad += msg.LaserSensor.angle_increment

    # update the map
    map_data = np.round(hits * 100 / (misses + hits)).astype(int)
    map.header.stamp = rospy.Time.now()
    map.data = map_data.ravel().tolist()
    rospy.loginfo("publishing map now ............")
    map_pub.publish(map)


##################################################################################
# callback functions
def move_callback(data):
    global move
    move = data


def laser_odom_callback(data):
    global laser_odom
    laser_odom = data
    draw_map(laser_odom)


if __name__ == "__main__":
    rospy.init_node("slam_node")
    rospy.loginfo("Start SLAM Node")
    rate = rospy.Rate(50)

    init_map()

    # input is velocity, orientation and laser data
    rospy.Subscriber(move_topic, TransformStamped, callback=move_callback)
    rospy.Subscriber(laser_odom_topic, LaserOdom, callback=laser_odom_callback)

    # output is the map
    map_pub = rospy.Publisher(map_topic, OccupancyGrid, queue_size=10)
    posPub = rospy.Publisher('/slam_loc', PoseStamped, queue_size=10)

    rospy.spin()
