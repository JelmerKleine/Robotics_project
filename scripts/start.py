#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import math
import time

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg

# Vehicle Properties (From vehicle source)
baseLinkOffset = [0, -1]

# overall length of the vehicle (meters)
length = 4.88

# overall wheel base of the vehicle (meters)
wheel_base = 2.85
# overhang
overh = (length - wheel_base)/2

# overall width
#width = 1.920
width = 1.910

# maximum steering angle (radians)
beta_max = 0.6

# minimum turning radius (meters)
# R_min = overh/math.sin(beta_max)
R_min = 5

# inner turning radius - minimum
Ri_min = np.sqrt(R_min**2 - wheel_base**2) - (width/2)

# outer turning radius - minimum
Re_min = np.sqrt((Ri_min+width)**2 + (wheel_base+overh)**2)

# minimum length of the parking space required - 1 shot
L_min = overh + np.sqrt(Re_min**2 - Ri_min**2)

# Variables
scan = Float32()
scanIncrement = Float32()
validCoordinatesBack = []
validCoordinatesFront = []


startPos = [0, 0]
currentVehiclePos = [Float64(), Float64()]
goalPos = [0, 0]
flag = 0

def getCurrentVehiclePos():
    return [currentVehiclePos[0].data, currentVehiclePos[1].data]

def laserCallback(msg):
    scan.data = msg.ranges
    scanIncrement.data = msg.angle_increment

def vehicleCallback(msg):
    currentVehiclePos[0].data = msg.pose.pose.position.x
    currentVehiclePos[1].data = msg.pose.pose.position.y
    

laser_data = rospy.Subscriber('/car/laser/scan',LaserScan,laserCallback)
vehicle_position = rospy.Subscriber('/ground_truth/state',Odometry,vehicleCallback)

  

def getYOffset(hypotenuse, angle):  # Opposite line
    return(math.sin(angle) * hypotenuse)


def getXOffset(hypotenuse, angle):  # Adjecent line
    return(math.cos(angle) * hypotenuse)


def getCoordinateFront(length, angle):
    return [baseLinkOffset[0] + getXOffset(length, angle), baseLinkOffset[1] + getYOffset(length, angle)]


def getCoordinateBack(length, angle):
    return [baseLinkOffset[0] + getXOffset(length, angle), baseLinkOffset[1] + -getYOffset(length, angle)]


def findObstacles(data):  # Lidar works from back and rotates counter clockwise
    # Calc angle between each scan
    count = scanIncrement.data
    angle = 0

    global validCoordinatesBack
    global validCoordinatesFront
    validCoordinatesFront = []
    validCoordinatesBack = []
    for length in data:  # Loop through data and find coordinate
        angle = angle + count
        if(length < 30):  # Check if laser actually hit something
            if(angle < (math.pi / 2)):  # Back 90 degrees
                cord = getCoordinateBack(length, ((math.pi / 2) - angle))
                validCoordinatesBack.append(cord)
            elif(angle > (math.pi / 2) and angle < math.pi):  # Front 90 degrees
                cord = getCoordinateFront(length, (angle - (math.pi / 2)))
                validCoordinatesFront.append(cord)
            

# No longer in use(splits at getting correct cords)
# def splitList(cords):
#     # Split cord in two list
#     prev = cords[0]
#     validList1 = []  # consists of top group of cords(obstacle 1)
#     validList2 = []  # consists of bottom group of cords(obstacle 2)
#     next = False
#     for cord in cords:
#         if((cord[1] - prev[1]) > -3 and not next):
#             validList1.append(cord)
#         elif(not next):
#             next = True
#             prev = cord
#         elif(next):
#             validList2.append(cord)
#         prev = cord
#     return validList1, validList2


def getCornersTop(obstacle):
    prev = obstacle[0]
    currenty = 0
    leftx = 0
    rightx = 0
    
    for point in obstacle:  # Find lowest y from list
        if(point[1] < prev[1]):
            currenty = point[1]
    
    for point in obstacle:  # Remove front portion of the back
        if(point[1] > (currenty + 2)):
            obstacle.remove(point)

    prev = obstacle[0]
    leftx = obstacle[0][0]
    for point in obstacle:  # Find lowest x from list
        if(point[0] < prev[0]):
            leftx = point[0]
    
    prev = obstacle[0]
    rightx = obstacle[0][0]
    for point in obstacle:  # Find highest x from list
        if(point[0] > prev[0]):
            rightx = point[0]

    return [[leftx, currenty], [rightx, currenty]]


def getYBottom(obstacle):
    prev = obstacle[0]
    currenty = obstacle[0][1]
    for point in obstacle:  # Find highest y from list
        if(point[1] > prev[1]):
            currenty = point[1]

    return currenty
        

def getObstacles():  # Obstacle 1 is behind car, obstacle 2 is in front of car
    obstacle2 = getCornersTop(validCoordinatesFront)
    obstacle1y = getYBottom(validCoordinatesBack)
    obstacle1 = [[obstacle2[0][0], obstacle1y], [obstacle2[1][0], obstacle1y]]
    return obstacle1, obstacle2


def calcTransition(obstacle1, obstacle2):  # Calculate the transition for parking
    r_prime = Ri_min + width/2
    
    c1 = [goalPos[0], goalPos[1] + r_prime]
    y_c2 = startPos[1] - r_prime

    pt = [
        (c1[1] + y_c2) / 2,
        0
    ]

    pt[1] = c1[1] + np.sqrt(r_prime**2 - (pt[0] - c1[0])**2)
    return pt


def parkVehicle(start_pos, transition_pos, goal_pos, update_pos):
    if update_pos[0]>start_pos[0]:
        vel = -12
        steer = 0
        
    elif update_pos[0]<=(start_pos[0]) and update_pos[0]>(transition_pos[0]):
        vel = -104
        steer = -0.49

    elif update_pos[0]<=(transition_pos[0]) and update_pos[0]>(goal_pos[0]-0.5) and flag == 0:
        steer = 0.49
        vel = -104
    elif update_pos[0]<=goal_pos[0]-0.5:
        steer = 0
        vel = 20
        global flag
        flag = 1
        
    elif flag == 1:
        steer = 0
        vel = -5

    return vel, steer  


def main():
    rospy.init_node('car', anonymous=True) #make node
    obstacle1 = 0
    obstacle2 = 0


    done = False
    while not rospy.is_shutdown():
        #print("Moving")
        if ((obstacle1 == 0) or (obstacle2 == 0) and not done):
            done = True
            findObstacles(scan.data)
            obstacle1, obstacle2 = getObstacles()
            print("obstacle1", obstacle1)
            print("obstacle2", obstacle2)
            startPos = getCurrentVehiclePos()
            print("startPos", startPos)
            goalPos = [(obstacle2[1][1] - obstacle1[1][1]) / 2, (obstacle1[1][0] - obstacle1[0][0]) / 2]
            print("goalPos", goalPos)
            print("currtentPos", getCurrentVehiclePos())
        else:
            continue
        velocity, steer = parkVehicle(startPos, calcTransition(obstacle1, obstacle2), goalPos, getCurrentVehiclePos())
        driveVehicle(velocity, steer)

            
        

def driveVehicle(vel_cmd,steer_cmd):
    print("Moving at velocity: ", vel_cmd, "and steer: ", steer_cmd, "!")
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
