#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from math import pi
import pandas as pd
import jenkspy
from networktables import NetworkTables, NetworkTablesInstance


NetworkTables.initialize("10.0.0.107")

table = NetworkTablesInstance.getDefault().getTable('crop laser scan table')



ANGLE = pi/7 #originally 5 
cropped_pub = None



#def finding_pole(ranges):
        
        #first_calculations = z/y
        #first_calculations * 100
        #if first_calculations >= .30 and first_calculations <= .70:
        #    if first_calculations > .50:
         #       distance_away = -x/2
          #      print("right")
           # elif first_calculations < .50:
            #    print("left")
             #   distance_away = x/2
            #else:
             #   print("Middle")

def callback(msg):
    HALF_ANGLE=ANGLE/2
    index_count = int(HALF_ANGLE//msg.angle_increment)
    ranges = msg.ranges #Collective of all points
    right_ranges = ranges[len(msg.ranges)-index_count:]
    left_ranges = ranges[:index_count]
    res_ranges = np.concatenate((right_ranges, left_ranges), -1)
    res_ranges = res_ranges.tolist()
    #res_ranges = res_rantenate((right_ranges, left_ranges), -1)
    #res_ranges = res_ranges[~np.isnan(res_ranges)]

    for i in res_ranges:
        if i >= 0.9144:
            i = float("nan")
    #finding_pole(res_ranges)
                #if i in right_ranges?
                #move right until i = 0
                #while i != 0:
                #   move.right() send command for robot to move, and will auto stop when i == 0 (position of pole?) 
		
        #This is to get the pole points to pass into getting_calculations
    new_res_ranges = [item for item in res_ranges if not(pd.isnull(item)) == True]
    #print(res_ranges)
    new_new_res_ranges = [*set(new_res_ranges)]
    grouping_ = {
        "Data":new_new_res_ranges
        #[
        #    1.0950000286102295, float('nan'), 1.0950000286102295, 1.1109999418258667, 1.1109999418258667, 1.1109999418258667, 1.1260000467300415, 1.1260000467300415, 1.1260000467300415, 1.1260000467300415, 1.1419999599456787, 1.1419999599456787, 1.1260000467300415, 1.0950000286102295, 1.0640000104904175, 1.0329999923706055, 1.0019999742507935, 0.9869999885559082, 0.9399999976158142, 0.9089999794960022, 0.9089999794960022, 0.925000011920929,1.2350000143051147, 1.25, 1.2660000324249268, 4.372000217437744, 4.309999942779541, 4.294000148773193, 2.430999994277954, 2.384000062942505, 2.36899995803833, 2.384000062942505, 2.4149999618530273, 2.384000062942505, 2.384000062942505, 2.305999994277954, 2.305999994277954, 2.259999990463257, 2.24399995803833, 6.5
        #]
    }
    #print(type(res_ranges))
    #print(type([1.0950000286102295, float('nan'), 1.0950000286102295, 1.1109999418258667, 1.1109999418258667, 1.1109999418258667, 1.1260000467300415, 1.1260000467300415, 1.1260000467300415, 1.1260000467300415, 1.1419999599456787, 1.1419999599456787, 1.1260000467300415, 1.0950000286102295, 1.0640000104904175, 1.0329999923706055, 1.0019999742507935, 0.9869999885559082, 0.9399999976158142, 0.9089999794960022, 0.9089999794960022, 0.925000011920929,1.2350000143051147, 1.25, 1.2660000324249268, 4.372000217437744, 4.309999942779541, 4.294000148773193, 2.430999994277954, 2.384000062942505, 2.36899995803833, 2.384000062942505, 2.4149999618530273, 2.384000062942505, 2.384000062942505, 2.305999994277954, 2.305999994277954, 2.259999990463257, 2.24399995803833, 6.5]))
    df = pd.DataFrame(grouping_)
    df.sort_values(by="Data")
    print(df["Data"])
    df["grouping"] = pd.qcut(df["Data"], q=6, labels=["Grouping1", "Grouping2", "Grouping3", "Grouping4", "Grouping5", "Grouping6"])
    #print(int(float(df["grouping"][0])))
    something = 1
    for i in len(df["grouping"]):
        string_numbers = str(something)
        new_df = df[df["grouping"]=="Grouping"+string_numbers]
        if len(new_df) <= 3:   
            new_df_mean = (new_df["Data"].mean())
            print(new_df_mean)
            middle_value = (len(new_df)-1)/2
        else:
            print("Grouping1 is greater than 4 floats")
            new_df["Data"] = float("nan")
            int(string_numbers)+1


    def getting_calculations():
        #get system to get point (p)
        #         #Have yet to get p(pole)
        #leftright = new_df_mean/2
        #leftright is the distance needed to square up to the pole, you get it by dividing the hypotunuse by 2
        #distance = new_df_mean/new_df_mean**3
        #ideal_distance = 1 #"placeholder for how far we would like the robot to move up towards the pole"
        #leftright=leftright*39.3701
        #distance=distance*39.3701
        #turning both value to inches for robot movement
        x = res_ranges.index(new_df["Data"].min()) 
        print(x)
        y = len(res_ranges)
        print(y)
        first_calculations = x/y
        first_calculations*100
        if first_calculations > .60:
            #distance_away = -x/2
            table.putString("Move Direction:", "left") #was once right
            print("left")
            print(first_calculations)
        elif first_calculations < .40:
            table.putString("Move Direction:","right") #was once left
            print("right")
            print(first_calculations)
        elif first_calculations > .40 and first_calculations < .60:
            print(first_calculations)
            table.putString("More Direction:", "Middle")
            print("Middle")
        print(len(new_df))
    getting_calculations()
        #return leftright, distance, ideal_distance - distance 
    msg.ranges = res_ranges
    #if right angle then
#finding_pole()


    if cropped_pub:
        cropped_pub.publish(msg)

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

