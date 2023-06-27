#!/usr/bin/env python2


import numpy as np
from math import pi
from sklearn.linear_model import LinearRegression
from networktables import NetworkTables, NetworkTablesInstance
import threading
from networktables import NetworkTables, NetworkTablesInstance
import sys



ANGLE = pi/4
LEN_MAX = 100
cropped_pub = None

print(sys.byteorder)

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize("192.168.1.98")


NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

print("Connected!")
table = NetworkTablesInstance.getDefault().getTable('laser_scan')


while True:

    #parse data

    #get angle_increment
    angle_increment = table.getNumber("angle_increment", 1)
    print(angle_increment)

    #get ranges
    ranges = table.getNumberArray("ranges", 0)

    if ranges == 0:
        print("no data for ranges")
        continue
    ranges = list(ranges)

    HALF_ANGLE = ANGLE/2
    index_count = int(HALF_ANGLE//angle_increment)
    #index_count = 3
    #ranges = [10,3,4,5,3,2,3,2,1,7,8,15]
    right_ranges = ranges[len(ranges)-index_count:]
    left_ranges = ranges[:index_count]
    res_ranges = np.concatenate((right_ranges, left_ranges), -1)

    #res_ranges = res_ranges[~np.isnan(res_ranges)]
    #msg.ranges = res_ranges
    ranges = res_ranges



    print("index number: " + str(len(ranges) - 1))
    print("total angle: " + str(ANGLE))

    #get the exact angle increment after cropping
    real_increment = ANGLE/(len(ranges) - 1)
    print("calculated index increment: " + str(real_increment))



    print(ranges)



    vector_x = [0]*len(ranges)
    vector_y = [0]*len(ranges)
    for i in range(len(ranges)):
        if np.isnan(ranges[i]):
            ranges[i] = LEN_MAX
            continue
        angle = (real_increment * i) - HALF_ANGLE + (np.pi/2)

        print("index: " + str(i) + " value: " + str(ranges[i]) + " angle: " + str(angle))


        #convert to (x, y)
        x = np.cos(angle) * ranges[i]
        y = np.sin(angle) * ranges[i]
        vector_x[i] = x
        vector_y[i] = y
        print("index: " + str(i) + " x: " + str(vector_x[i]) + " y: " + str(vector_y[i]))

    #linear regression
    array_x = np.asarray(vector_x).reshape((-1, 1))
    array_y = np.asarray(vector_y)

    model = LinearRegression()

    model.fit(array_x, array_y)

    r_sq = model.score(array_x, array_y)
    print("linear regression complete")
    print("coefficient of determination: " + str(r_sq))
    print("intercept: " + str(model.intercept_))

    slope = model.coef_[0]

    print("slope: " + str(slope))


    line_angle = np.arctan(slope)

    angle_to_move = -line_angle
    print("angle to move the robot (in degrees): " + str(angle_to_move*180/pi))

    #publish move angle to network table

    table.putNumber("angle_to_move", angle_to_move)







