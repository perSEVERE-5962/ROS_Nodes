#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from math import pi
import pandas as pd
import jenkspy
from networktables import NetworkTables, NetworkTablesInstance
import tkinter as tk
from math import cos, tan
NetworkTables.initialize("10.0.0.107")

table = NetworkTablesInstance.getDefault().getTable('crop laser scan table')
global intricate_switch
root = tk.Tk()
button = tk.Button(root, text="Switch", width=15, command=intricate_switch)
ANGLE = pi/7 #originally 5 
cropped_pub = None

def callback(msg):
    HALF_ANGLE=ANGLE/2
    index_count = int(HALF_ANGLE//msg.angle_increment)
    ranges = msg.ranges #Collective of all points0
    right_ranges = ranges[len(msg.ranges)-index_count:]
    left_ranges = ranges[:index_count]
    res_ranges = np.concatenate((left_ranges, right_ranges), -1)
    res_ranges = res_ranges.tolist()
    #res_ranges = res_rantenate((right_ranges, left_ranges), -1)
    #res_ranges = res_ranges[~np.isnan(res_ranges)]
    for i in res_ranges:
        if i >= 0.9144:
            i = float("nan")
    if cropped_pub:
        msg.ranges = res_ranges
        cropped_pub.publish(msg)

    angle_increment = msg.angle_increment
    angle = angle
    new_list = []
    for i,z in enumerate(res_ranges):
        #new_list.append(x+angle_increment)
        something = angle_increment*i
        new_list.append(angle+something)
    number = 0
    first_new_list = []
    second_new_list = []
    for i in res_ranges:
            y = cos(new_list[number])
            answer = y*i
            second_new_list.append(answer)
            number+1
    print(second_new_list)
    new_number = 0
    for i in res_ranges:
        sine = sin(new_list[number])
        new_answer = sine*i
        first_new_list.append(new_answer)
        new_number+1

    import numpy as np
    from sklearn.cluster import KMeans
    import matplotlib.pyplot as plt
    import pandas as pd
    kmeans = KMeans(n_clusters=4)
    data = {
        "x": first_new_list,
        "y": second_new_list 
    }

    df = pd.DataFrame(data)
    df["Specified_Category"] = kmeans.fit_predict(df)
    df["Specified_Category"] = df["Specified_Category"].astype("category")
    print(df)
    plt.scatter(data["x"], data["y"])
    plt.show()



    #finding_pole(res_ranges)
                #if i in right_ranges?
                #move right until i = 0
                #while i != 0:
                #   move.right() send command for robot to move, and will auto stop when i == 0 (position of pole?) 
		
        #This is to get the pole points to pass into getting_calculations
    



if __name__ =='__main__':
    try:
        rospy.init_node('crop_laser_scan')

        rospy.Subscriber('/scan', LaserScan, callback)
        cropped_pub = rospy.Publisher('/scan_cropped', LaserScan, queue_size=1)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass

#________________________________________________________________________________
#Jenks Natural Breaks Optimization

