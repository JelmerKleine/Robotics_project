#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import math

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

def callback(msg):
    #print(len(msg.ranges)) len is 2019 from 0-360
    scan.data = msg.ranges
    
    
scan =Float32()
laser_data = rospy.Subscriber('/car/laser/scan',LaserScan,callback)

def main():
    rospy.init_node('car', anonymous=True) #make node
   
    while not rospy.is_shutdown(): 
	print("moving")
        speed = 0
        steer = 0
        driveVehicle(speed,steer) 
        scannerData()

def driveVehicle(vel_cmd,steer_cmd):
    wheel_rear_right = rospy.Publisher('/car/rr_wheel_controller/command', Float64, queue_size=10)
    wheel_rear_left = rospy.Publisher('/car/rl_wheel_controller/command', Float64, queue_size=10)
    steer_front = rospy.Publisher('/car/fl_steer_controller/command', Float64, queue_size=10)
    steer_rear = rospy.Publisher('/car/fr_steer_controller/command', Float64, queue_size=10)
    
    #rospy.loginfo(vel_cmd)
    #rospy.loginfo(steer_cmd)
    wheel_rear_right.publish(vel_cmd)
    wheel_rear_left.publish(vel_cmd)
    steer_front.publish(steer_cmd)
    steer_rear.publish(steer_cmd)
    rate = rospy.Rate(10) # 100hz
    rate.sleep()


    



def scannerData():
    print(scan)
    laser_data = rospy.Subscriber('/car/laser/scan',LaserScan,callback)

if __name__ == "__main__":
    
    main()    
