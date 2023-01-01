#!/usr/bin/env python3

import tf
import rospy
import numpy as np
from skimage.draw import line
from nav_msgs.msg import OccupancyGrid
from robotix.msg import LaserOdom

map_topic = "/mapping"
laser_odom_topic = "/sensors_incorporating"

map = OccupancyGrid()
hits, misses, map_data = np.ndarray, np.ndarray, np.ndarray

pubObj = None
subObj = None

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


# return them in radians
def euler_angles(x, y, z, w):
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([x, y, z, w])
    return roll, pitch, yaw


def get_line_start(world):
    global map
    # Get the World
    start_x = world.OdometrySensor.pose.pose.position.x
    start_y = world.OdometrySensor.pose.pose.position.y

    # Relative to the Map, CELL
    start_i = int(round((start_x) / map.info.resolution) + map.info.width / 2)
    start_j = int(round((start_y) / map.info.resolution) + map.info.height / 2)

    yaw = euler_angles(
        world.OdometrySensor.pose.pose.orientation.x,
        world.OdometrySensor.pose.pose.orientation.y,
        world.OdometrySensor.pose.pose.orientation.z,
        world.OdometrySensor.pose.pose.orientation.w,
    )[2]

    return start_x, start_y, start_i, start_j, yaw


def get_line_end(start_x, start_y, cur_reading, cur_angle):
    global map
    # Relative to the World
    end_x = start_x + cur_reading * np.cos(cur_angle)
    end_y = start_y + cur_reading * np.sin(cur_angle)

    # Relative to the Map, CELL
    end_i = int(round((end_x) / map.info.resolution) + map.info.width / 2)
    end_j = int(round((end_y) / map.info.resolution) + map.info.height / 2)

    return end_i, end_j


# x, y are world coordinates
# i, j are map coordinates (i.e cell)
def draw_map(msg):
    global map, hits, misses, map_data, pubObj
    rospy.loginfo("Start publishing")

    # get the line start point on the map
    start_x, start_y, start_i, start_j, yaw = get_line_start(msg)

    readings = msg.LaserSensor.ranges
    angle_rad = msg.LaserSensor.angle_min

    # skip the current reading in 3 cases
    for i in range(len(readings)):
        # 1. if it's outise the laser ranges (min to max)
        if readings[i] < msg.LaserSensor.range_min or readings[i] > msg.LaserSensor.range_max:
            angle_rad += msg.LaserSensor.angle_increment
            continue

        # get the line end point on the map
        cur_angle = angle_rad + yaw
        end_i, end_j = get_line_end(start_x, start_y, readings[i], cur_angle)

        # draw the line, those are the cells that are occupied by the laser drawn from start to end
        beam_y, beam_x = line(r0=start_j, c0=start_i, r1=end_j, c1=end_i)

        # mark whatever laser cell as a miss if it's inisde the map boundary
        # and skip the ones that are outside the map boundary
        for j in range(len(beam_x) - 1):
            if beam_x[j] >= map.info.width or beam_y[j] >= map.info.height:
                continue
            misses[beam_y[j], beam_x[j]] += 1

        # 2. check if the beam end cell is NOT inside the map boundary as well
        # then just update the angle for the next reading and skip
        if beam_x[-1] >= map.info.width or beam_y[-1] >= map.info.height:
            angle_rad += msg.LaserSensor.angle_increment
            continue

        # otherwise it's inside, we need to check just if its a max reading or not
        # if the cur_reading is NOT a max range reading, consider it a hit
        if readings[i] != msg.LaserSensor.range_max:
            hits[beam_y[-1], beam_x[-1]] += 1
        else:
            misses[beam_y[-1], beam_x[-1]] += 1

        # 3. just go to the next angle anyway
        angle_rad += msg.LaserSensor.angle_increment

    # add the hits and misses to the map data and publish them
    map_data = np.round(hits * 100 / (misses + hits)).astype(int)
    map.header.stamp = rospy.Time.now()
    map.data = map_data.ravel().tolist()
    rospy.loginfo("publishing map now ............")
    pubObj.publish(map)


if __name__ == "__main__":
    # global pubObj, subObj
    rospy.init_node("mapping")
    rospy.loginfo("Start Mapping Node")
    rate = rospy.Rate(50)

    init_map()

    pubObj = rospy.Publisher(map_topic, OccupancyGrid, queue_size=10)
    subObj = rospy.Subscriber(laser_odom_topic, LaserOdom, callback=draw_map, queue_size=1)

    rospy.spin()
