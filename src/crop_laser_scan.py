#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib
import numpy as np
from math import pi
import math
import pandas as pd
import jenkspy
from networktables import NetworkTables, NetworkTablesInstance
from math import cos, tan, sin
#Changing the Clustering Algo is: 
NetworkTables.initialize("10.0.0.107")
table = NetworkTablesInstance.getDefault().getTable('crop laser scan table')
ANGLE = pi/5 #originally 5 
cropped_pub = None
global angle
def callback(msg):
    HALF_ANGLE=ANGLE/2
    index_count = int(HALF_ANGLE//msg.angle_increment)
    ranges = msg.ranges #of all points0
    right_ranges = ranges[len(msg.ranges)-index_count:]
    left_ranges = ranges[:index_count]
    res_ranges = np.concatenate((right_ranges, left_ranges), -1)
    res_ranges = res_ranges.tolist()
    message_range = msg.ranges

    #res_ranges = res_rantenate((right_ranges, left_ranges), -1)
    #res_ranges = res_ranges[~np.isnan(res_ranges)]
    for i in range(len(res_ranges)):
        if res_ranges[i] >= 1.5:
            res_ranges[i] = float("nan")
    #0.9144
    if cropped_pub:
        msg.ranges = res_ranges
        cropped_pub.publish(msg)
    angle = -HALF_ANGLE
    angle_increment = msg.angle_increment
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
            number=number+1
    new_number = 0
    for i in res_ranges:
        sine = sin(new_list[new_number])
        new_answer = sine*i
        first_new_list.append(new_answer)
        new_number=new_number+1
    from sklearn.cluster import KMeans
    list_of_removal = []
    import matplotlib.pyplot as plt
    kmeans = KMeans(n_clusters=4)
    #__
    first_new_list = [item for item in first_new_list if not(math.isnan(item)) == True]
    second_new_list = [item for item in second_new_list if not(math.isnan(item)) == True]   
 

    #x = np.where(np.isnan(first_new_list))
    #for i in x:
     #   print(i)
     #   list_of_removal.append(i+1)
    #for i in list_of_removal:
     #   pass
        
    #_________ first_new, second_new, new_list
    data = {
        "x": first_new_list,
        "y": second_new_list 
        }
    #_______________________
    #______________________
    color = []
    df = pd.DataFrame(data)
    df["Specified_Category"] = kmeans.fit_predict(df)
    df["Specified_Category"] = df["Specified_Category"].astype("category")
    for i in df["Specified_Category"]:
        #print(i)
        if i == 0:color.append("red")
        elif i == 1:color.append("blue")
        elif i == 2:color.append("green")
        elif i == 3:color.append("black")
        elif i == 4:color.append("purple")
        elif i == 5:color.append("pink")
    centeroids = kmeans.cluster_centers_
    #plt.scatter(data["x"], data["y"], c=color)
    #plt.savefig("clustering_graph.png")
    #plt.clf()
    print("here is")
    centeroids = centeroids.tolist()
    x_centeroid=[]
    y_centeroid=[]
    for i in centeroids:
        x_centeroid.append(i[0])
        y_centeroid.append(i[1])
    real_centeroids_calculations = []
    for i in centeroids:
        real_centeroids_calculations.append(abs(i[0])+abs(i[1]))
    xyz = min(real_centeroids_calculations)
    location=real_centeroids_calculations.index(xyz)
    print("HERE IS")
    centeroid_number = 0
    cenetroid_x_value = []
    cenetroid_y_value = []
    color2 = []
    for i in centeroids:
        cenetroid_x_value.append(centeroids[centeroid_number][0])
        cenetroid_y_value.append(centeroids[centeroid_number][1])
        if centeroid_number == location:
            color2.append("red")
        elif centeroid_number != location:
            color2.append("black")
        centeroid_number=centeroid_number+1



    
    plt.scatter(cenetroid_x_value, cenetroid_y_value, c=color2)
    plt.savefig("centeroids.png")
    plt.clf()
    plt.scatter(data["x"], data["y"],c=color)
    plt.savefig("clustering_graph.png")
    plt.clf()
    print(centeroids[location][0])
    print(centeroids[location][1])
    if centeroids[location][0] > -0.05 and centeroids[location][0]< 0.05:
        print('X Perfect')
        if centeroids[location][1] > .9 and centeroids[location][1] < 1.1:
            print("Y Perfect")
        elif centeroids[location][1] < 1:
            print("Back")
        elif centeroids[location][1] > 1:
            print("Forward")   
    elif centeroids[location][0] < 0:
        print('go right')    
    elif centeroids[location][0] > 0:
        print('go left')
    
    
    
        

        



    #print(res_ranges, new_list)
#Still Have to Fine Tune




if __name__ =='__main__':
    try:
        rospy.init_node('crop_laser_scan')

        rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
        cropped_pub = rospy.Publisher('/scan_cropped', LaserScan, queue_size=1)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass