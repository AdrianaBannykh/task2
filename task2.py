#! /usr/bin/python3

import math
import rospy
from sensor_msgs.msg import LaserScan

pub_before = rospy.Publisher('/laser_scan_before_filter', LaserScan, queue_size = 10)
pub_after = rospy.Publisher('/laser_scan_after_filter', LaserScan, queue_size = 10)

def callback(msg):
    pub_before.publish(msg)
    msg.ranges = get_emissions(msg.ranges, msg.angle_min, msg.angle_increment)
    pub_after.publish(msg)
def get_coords(r, theta):
    return r * math.cos(theta), r * math.sin(theta)
def get_angle(angle_min, angle_increment, position):
    return angle_min + angle_increment * position
def get_distance(a, b):
    return math.sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2))
def get_emissions(emissions, angle_min, angle_increment):
    emissions = []
    step = 7
    max_distance = 0.7
    max_filter_value = 7000.0
    for i in range(step):
        emissions.append(max_filter_value)
    for i in range(step, len(data) - step):
        before_point = get_coords(data[i - step], get_angle(angle_min, angle_increment, i - step))
        current_point = get_coords(data[i], get_angle(angle_min, angle_increment, i))
        after_point = get_coords(data[i + step], get_angle(angle_min, angle_increment, i + step))
        if get_distance(before_point, current_point) > max_distance or get_distance(after_point, current_point) > max_distance:
            emissions.append(max_filter_value)
        else:
            emissions.append(data[i])
    for i in range(step):
        emissions.append(max_filter_value)
    return emissions

rospy.init_node('task2_laser_scan')
rospy.Subscriber('base_scan', LaserScan, callback)
r = rospy.Rate(0.5)
while not (rospy.is_shutdown()):
    r.sleep()