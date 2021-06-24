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

scan = Float32()
scanIncrement = Float32()
validCoordinates = []

def callback(msg):
    scan.data = msg.ranges
    scanIncrement.data = msg.angle_increment
    

laser_data = rospy.Subscriber('/car/laser/scan',LaserScan,callback)

lidarPos = [0, 0]

def getYOffset(hypotenuse, angle):  # Opposite line
    return(math.sin(90 - angle) * hypotenuse)


def getXOffset(hypotenuse, angle):  # Adjecent line
    return(math.cos(90 - angle) * hypotenuse)


def getCoordinate(length, angle):
    return [lidarPos[0] + getXOffset(length, angle), lidarPos[1] + getYOffset(length, angle)]


def findObstacles(data):
    # Calc angle between each scan
    count = scanIncrement.data
    angle = 0

    global validCoordinates
    validCoordinates = []
    for item in data:  # Loop through data and find coordinate
        angle = angle + count
        if(item < 30):  # Check if laser actually hit something
            validCoordinates.append(getCoordinate(item, angle))




def main():
    rospy.init_node('car', anonymous=True) #make node
    
    while not rospy.is_shutdown():
        print("Moving")
        speed = 0
        steer = 0
        driveVehicle(speed, steer)
        findObstacles(scan.data)
        if (len(validCoordinates) is not 0):
            print(validCoordinates[0])


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


if __name__ == "__main__":
    main()    
