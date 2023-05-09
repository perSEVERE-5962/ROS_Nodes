#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from math import pi

ANGLE = pi/4
cropped_pub = None


def callback(msg):
    HALF_ANGLE=ANGLE/2
    index_count = int(HALF_ANGLE//msg.angle_increment)
    #index_count = 3
    ranges = msg.ranges
    #ranges = [2,3,4,5,3,2,3,2,1,7,8,5]
    right_ranges = ranges[len(ranges)-index_count:]
    left_ranges = ranges[:index_count]
    res_ranges = np.concatenate((right_ranges, left_ranges), -1)
    #res_ranges = res_ranges[~np.isnan(res_ranges)]
    msg.ranges = res_ranges

    if cropped_pub:
        cropped_pub.publish(msg)

    #find largest element
    largest_val = 0
    largest_idx = 0
    for i in range(0, len(ranges) - 1):
        if ranges[i] > largest_val:
            largest_val = ranges[i]
            largest_idx = i

    print("largest distance: " + str(largest_val))
    print("the index of ranges that it is located at: " + str(largest_idx))


msg = None
callback(msg)

if __name__ =='__main__':
    try:
        rospy.init_node('crop_laser_scan')

        rospy.Subscriber('/scan', LaserScan, callback)
        cropped_pub = rospy.Publisher('/scan_cropped', LaserScan, queue_size=1)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


