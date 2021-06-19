#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import math

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

def main():
    rospy.init_node('car', anonymous=True) #make node
   
    while not rospy.is_shutdown():    
         driveVehicle() 

def driveVehicle():
    wheel_rear_right = rospy.Publisher('car/rr_wheel_controller/command', Float64, queue_size=10)
    wheel_rear_left = rospy.Publisher('car/rl_wheel_controller/command', Float64, queue_size=10)
    steer_front = rospy.Publisher('/car/fl_steer_controller/command', Float64, queue_size=10)
    steer_rear = rospy.Publisher('/car/fr_steer_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(10) # 100hz
    #while not rospy.is_shutdown():
    
    print("hallo")
    vel_cmd = 200
    steer_cmd= 20000
    rospy.loginfo(vel_cmd)
    rospy.loginfo(steer_cmd)
    wheel_rear_right.publish(vel_cmd)
    wheel_rear_left.publish(vel_cmd)
    steer_front.publish(steer_cmd)
    steer_rear.publish(steer_cmd)
    rate.sleep()

if __name__ == "__main__":
    
    main()    
