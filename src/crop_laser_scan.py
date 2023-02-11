#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from math import pi

ANGLE = pi/5
cropped_pub = None



def finding_pole(ranges):
        x = min(ranges)
        y = len(ranges)
        z = front_points.index(x)
        first_calculations = z/y
        first_calculations * 100
        if first_calculations >= .30 and first_calculations <= .70:
	        print("Worked!")
            if first_calculations > .50:
	            print("right")
		        distance_away = -x/2
	        elif first_calculations < .50:
		        print("left")
		        distance_away = x/2
	        else:
		        print("Middle")

def callback(msg):
    HALF_ANGLE=ANGLE/2
    index_count = int(HALF_ANGLE//msg.angle_increment)
    ranges = msg.ranges
    right_ranges = ranges[len(msg.ranges)-index_count:]
    left_ranges = ranges[:index_count]
    res_ranges = np.concatenate((right_ranges, left_ranges), -1)
    #res_ranges = res_ranges[~np.isnan(res_ranges)]
    for i in res_ranges:
        if i >= 0.9144:
            i = float("nan")
    finding_pole(res_ranges)
                #if i in right_ranges?
                #move right until i = 0
                #while i != 0:
                #   move.right() send command for robot to move, and will auto stop when i == 0 (position of pole?) 
		
        #This is to get the pole points to pass into getting_calculations
        
    def getting_calculations():
        pass
        #get system to get point (p)
        p = 0.6096 # Stand in for the point representing the center of the pole
        leftright = p/2
        #leftright is the distance needed to square up to the pole, you get it by dividing the hypotunuse ⊿ by 2
        distance = p/p**3
        ideal_distance = 1 #"placeholder for how far we would like the robot to move up towards the pole"
        leftright=leftright*39.3701
        distance=distance*39.3701
        #turning both value to inches for robot movement
        return leftright, distance, ideal_distance - distance 
    msg.ranges = res_ranges
    #if right angle then 

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


