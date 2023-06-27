#!/usr/bin/env python2
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
import threading
from networktables import NetworkTables, NetworkTablesInstance
from math import pi
import sys

NetworkTables.initialize('192.168.1.98')


ANGLE = pi/4
LEN_MAX = 100
cropped_pub = None


table = NetworkTablesInstance.getDefault().getTable('laser_scan')

#laser_scan_topic = table.getEntry('laser_scan')


def callback(msg):
    '''print(msg.angle_min)
    print(msg.angle_max)
    data = bytearray(struct.pack(">f", np.float64(msg.angle_min)))
    data += bytearray(struct.pack(">f", np.float64(msg.angle_max)))
#    print(len(data))
    data += bytearray(struct.pack(">f", msg.angle_increment))
    data += bytearray(struct.pack(">f", msg.time_increment))
    data += bytearray(struct.pack(">f", msg.scan_time))
    data += bytearray(struct.pack(">f", msg.range_min))
    data += bytearray(struct.pack(">f", msg.range_max))
    data += bytearray(struct.pack(">i", len(msg.ranges)))
    for point in msg.ranges:
        data += bytearray(struct.pack(">f", point))
    data += bytearray(struct.pack(">i", len(msg.intensities)))
    for intensity in msg.intensities:
        data += bytearray(struct.pack(">f", intensity))

    #rospy.loginfo(data)
    table.putRaw("laser_scan", data)'''

    angle_increment = msg.angle_increment
    table.putNumber("angle_increment", angle_increment)

    ranges = msg.ranges

    #crop ranges

    HALF_ANGLE = ANGLE / 2
    index_count = int(HALF_ANGLE // angle_increment)

    right_ranges = ranges[len(ranges) - index_count:]
    left_ranges = ranges[:index_count]
    res_ranges = np.concatenate((right_ranges, left_ranges), -1)

    ranges = res_ranges
    table.putNumberArray("ranges", ranges)
    print(ranges)

    #get angle to move

    angle_to_move = table.getNumber("angle_to_move", "[none returned]")
    print("angle to move: " + str(angle_to_move*180/pi))


if __name__ =='__main__':
    try:
        rospy.init_node('scan_2_net_tab')

        rospy.Subscriber('/scan', LaserScan, callback)
        cropped_pub = rospy.Publisher('/scan_cropped', LaserScan, 1)


        rospy.spin()

    except rospy.ROSInterruptException:
        pass


