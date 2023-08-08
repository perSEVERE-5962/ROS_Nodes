#!/usr/bin/env python2
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

import threading
from networktables import NetworkTables, NetworkTablesInstance
from math import pi
import sys

from sklearn.linear_model import LinearRegression

#print(sys.byteorder)

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()



NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

'''
with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

print("Connected!")
table = NetworkTablesInstance.getDefault().getTable('laser_scan')
'''

rospy.init_node('scan_2_net_tab')
pub = rospy.Publisher('/scan_cropped', LaserScan, queue_size=10)
laser_scan = LaserScan()

# laser_scan_topic = table.getEntry('laser_scan')

ANGLE = pi/4
LEN_MAX = 100
cropped_pub = None


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

    # publish cropped scan
    current_time = rospy.Time.now()
    laser_scan.header.stamp = current_time
    laser_scan.header.frame_id = 'base_laser'
    laser_scan.angle_min = -HALF_ANGLE
    laser_scan.angle_max = HALF_ANGLE
    laser_scan.angle_increment = angle_increment
    laser_scan.time_increment = 4.99999987369e-05
    laser_scan.range_min = 0.00000001
    laser_scan.range_max = 32.0
    laser_scan.ranges = ranges






    #print("index number: " + str(len(ranges) - 1))
    #print("total angle: " + str(ANGLE))

    #get the exact angle increment after cropping
    real_increment = ANGLE/(len(ranges) - 1)
    #print("calculated index increment: " + str(real_increment))



    #print(ranges)



    vector_x = [0]*len(ranges)
    vector_y = [0]*len(ranges)
    for i in range(len(ranges)):
        if np.isnan(ranges[i]):
            ranges[i] = LEN_MAX
            continue
        angle = (real_increment * i) - HALF_ANGLE + (np.pi/2)

        #print("index: " + str(i) + " value: " + str(ranges[i]) + " angle: " + str(angle))


        #convert to (x, y)
        x = np.cos(angle) * ranges[i]
        y = np.sin(angle) * ranges[i]
        vector_x[i] = x
        vector_y[i] = y
        #print("index: " + str(i) + " x: " + str(vector_x[i]) + " y: " + str(vector_y[i]))

    #linear regression
    array_x = np.asarray(vector_x).reshape((-1, 1))
    array_y = np.asarray(vector_y)

    model = LinearRegression()

    model.fit(array_x, array_y)

    r_sq = model.score(array_x, array_y)
    #print("linear regression complete")
    print("coefficient of determination: " + str(r_sq))
    #print("intercept: " + str(model.intercept_))

    slope = model.coef_[0]

    print("slope: " + str(slope))


    line_angle = np.arctan(slope)

    angle_to_move = -line_angle
    #print("angle to move the robot (in degrees): " + str(angle_to_move*180/pi))

    #publish move angle to network table

    print("angle to move: ")
    if isinstance(angle_to_move, str):
        print(angle_to_move)
    else:
        print(str(angle_to_move * (180 / pi)))

    table.putNumber("angle_to_move", angle_to_move)

    pub.publish(laser_scan)




if __name__ =='__main__':
    try:

        rospy.init_node('scan_2_net_tab')

        rospy.Subscriber('/scan', LaserScan, callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass




