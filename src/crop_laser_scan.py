#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from math import pi

ANGLE = pi/4
MAX_WALL_DIST = 1
cropped_pub = None


def callback(msg):
    HALF_ANGLE=ANGLE/2
    index_count = int(HALF_ANGLE//msg.angle_increment)
    #index_count = 3
    ranges = msg.ranges
    #ranges = [10,3,4,5,3,2,3,2,1,7,8,15]
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

    print("index increment: " + str(msg.angle_increment))
    print("index number: " + str(len(ranges) - 1))
    print("total angle: " + str(ANGLE))
    print("calculated index increment: " + str((ANGLE/len(ranges)) - 1))
    print("largest distance: " + str(largest_val))
    print("the index of ranges that it is located at: " + str(largest_idx))
    print(ranges)

    #find closest numerical datapoint to the first one
    i = 0
    while np.isnan(ranges[i]):
        i += 1
        if i >= len(ranges):
            print("There is no numerical data")
            break

    a = ranges[i]
    print("first numerical index: " + str(i))

    i = len(ranges) - 1
    while np.isnan(ranges[i]):
        i -= 1
        if i < 0:
            print("There is no numerical data")
            break
    b = ranges[i]
    print("last numerical index: " + str(i))



    C = HALF_ANGLE * 2


    #calculate length of c using law of cosines
    c = np.sqrt(np.square(a) + np.square(b) - (2 * a * b * np.cos(C)))

    #calculate smaller angle
    if a < b:
        #calculate angle A
        A = np.arcsin((a * np.sin(C)/c))
        #calculate angle B
        B = pi - A - C
    else:
        # calculate angle B
        B = np.arcsin((b * np.sin(C) / c))
        # calculate angle A
        A = pi - B - C

    #calculate distance to rotate to square up the robot
    x = B - A

    print("Left distance: " + str(a))
    print("Right distance: " + str(b))
    print("Distance between the points at which they hit the wall: " + str(c))
    print("left distance to wall angle: " + str(B * 180 / pi))
    print("right distance to wall angle: " + str(A * 180 / pi))
    print("angle to rotate to robot to square it up: " + str(x * 180 / pi))




if __name__ =='__main__':
    try:
        rospy.init_node('crop_laser_scan')

        rospy.Subscriber('/scan', LaserScan, callback)
        cropped_pub = rospy.Publisher('/scan_cropped', LaserScan, queue_size=1)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


