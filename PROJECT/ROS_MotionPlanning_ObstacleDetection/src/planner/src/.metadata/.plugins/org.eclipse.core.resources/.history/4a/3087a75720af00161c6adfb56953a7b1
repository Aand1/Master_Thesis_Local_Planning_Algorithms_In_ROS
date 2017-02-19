#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from copy import copy, deepcopy
from nav_msgs.msg import Odometry
import tf
import math
import sys
from geometry_msgs.msg import Twist
from gnomevfs._gnomevfs import Drive
from __builtin__ import False

# defining the global variables
REACH_GOAL=False
data_range = 0
goal_point_x = -0
goal_point_y = -0
YAW_FLAG = 0
PLANNER_FLAG = 0  # if equal to 0 GLOBAL_PLANNER is working else LOCAL_PLANNER is working
yaw_offset = 0.3  # offset to the yaw angle in which we have to reduce error
lp_offset = 1.5  # this offset declare the range in whoch the global planner should be off
prefix = sys.argv[1:1]  # prefix for the topics
obstacle_angle = 5  # obstacle range in angle points 
obstacle_range = obstacle_angle / 0.5    
angleDiv = 3.14 / 360  # in if the obstacle in this range will be detected then local planner is launched.
start = True
AVOID_FLAG = False
goal_angle_right = 0
goal_angle_left = 0
compute_currentYAW = 0
compute_currentX = 0
compute_currentY = 0

def driveStop():
    
    pub = rospy.Publisher('pioneer1/cmd_vel', Twist)
    twist = Twist()
    for i in range(0, 60):
        if i < 30:
            twist.linear.x = 0.1
        else: 
            twist.linear.x = 0.0
    print "STOP!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"        
    
    pub.publish(twist)
    
def tf_QuatQartesian(dataX, dataY, dataZ, dataYaw):
    quaternion = (dataX, dataY, dataZ, dataYaw)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    return yaw

def twistYaw(goalYaw, currentYaw):
    global YAW_FLAG
    pub = rospy.Publisher('pioneer1/cmd_vel', Twist)
    # rospy.init_node('twistYaw')
    twist = Twist()
    # driveStop()
    # goalYaw=goalYaw+3.14
    # currentYaw=currentYaw+3.14
    #print "GOAL_YAW:" + str(goalYaw)
    #print "CURRNET YAW:" + str(currentYaw)
    if goalYaw > currentYaw:
        twist.angular.z = 0.1
        #print "Turning left"
        if goalYaw - 0.01 < currentYaw:
            twist.angular.z = 0.0  
            YAW_FLAG = 1  
            #print "Turning left stopped, yaw reached"
    if goalYaw < currentYaw:
        twist.angular.z = -0.1
        #print "Turning right"
        if goalYaw + 0.01 > currentYaw:
            twist.angular.z = 0.0 
            YAW_FLAG = 1
            #print "Turning right stopped, yaw reached"
    pub.publish(twist)
    
def twistStop():
    pub = rospy.Publisher('pioneer1/cmd_vel', Twist)
    # rospy.init_node('twistYaw')
    twist = Twist()
    
    twist.angular.z = 0.00 
            
    pub.publish(twist)
    
def driveToPoint(goalP, currentP):
    global start
    global REACH_GOAL
    pub = rospy.Publisher('pioneer1/cmd_vel', Twist)
    twist = Twist()
    # print "Drive"
    # twistStop()
    REACH_GOAL=0
    if start == True:
        
        for i in range(0, 60):
            if i < 30:
                twist.linear.x = 0.1
            else:
                twist.linear.x = 0.2
                start = False
        
    if goalP > currentP or goalP < currentP:
            twist.linear.x = 0.0
            #REACH_GOAL=True
            #print "Goal reached"
            if goalP - 0.02 > currentP or goalP + 0.02 < currentP:
                twist.linear.x = 0.1
                if goalP - 0.2 > currentP or goalP + 0.2 < currentP:
                    twist.linear.x = 0.2
                    if (goalP - 1.0 > currentP) or (goalP + 1.0 < currentP):
                        twist.linear.x = 0.2
                        
    pub.publish(twist)

def getGoalYaw(startX, startY, goalX, goalY):
    # compute the yaw between start point and global point
    goal_yaw = math.atan2((goalY - startY), (goalX - startX))
    return goal_yaw

def laser_data(data):
    # local_planner definition
    global PLANNER_FLAG
    global data_range
    laser_data = data.ranges  # get laser data in frame
    data_range = list(laser_data)  # convert laser data to the list
    
    for i in range(0, len(data.ranges)):
        if laser_data[i] < lp_offset:
            data_range[i] = 1
            if PLANNER_FLAG == 0:
                if i >= ((len(data.ranges) / 2 - 1) - obstacle_range / 2) and i <= ((len(data.ranges) / 2 - 1) + obstacle_range / 2):
                    PLANNER_FLAG = 1
        else:
            data_range[i] = 0
    #print data_range
def global_planner(yaw, goal_yaw, current_x, current_y):
    global YAW_FLAG
    print "print global planner is working"  
    print goal_point_x
    print goal_point_y
    print current_x
    print current_y  
    if goal_yaw + yaw_offset > yaw or goal_yaw - yaw_offset < yaw:
        YAW_FLAG = 0
    if YAW_FLAG == 0:
        twistYaw(goal_yaw, yaw)
    if YAW_FLAG == 1:
        driveToPoint(goal_point_y, current_y)
        
def computeYaw(direction, goalPoint, yaw):
    
    if direction == 0:
        #print "should compute to left"
        goalYaw = compute_currentYAW + angleDiv * (goalPoint + obstacle_range)
    if direction == 1:
        #print "should compute to right"
        goalYaw = compute_currentYAW - angleDiv * (goalPoint + obstacle_range)
    return goalYaw

def local_planner(yaw, currentX, currentY):
    # print "Avoid obstacle"
    # print data_range
    print "local planner is working"
    global YAW_FLAG
    global compute_currentYAW
    global compute_currentX
    global compute_currentY
    global AVOID_FLAG
    global PLANNER_FLAG
    global goal_angle_right
    global goal_angle_left
    print "Current point on X:" + str(currentX) + "on Y" + str(currentY)
    driveStop()
    free_iterator_right = 0
    free_iterator_left = 0
    print data_range
    direction = 1  # 0-LEFT 1-RIGHT

    if AVOID_FLAG == False:
        
        for i in range(0, (len(data_range) / 2)):
            if data_range[(len(data_range) / 2 - 1) + i] == 0:
                free_iterator_left = free_iterator_left + 1
                # print free_iterator_left
                if free_iterator_left == 2 * (obstacle_range + obstacle_angle):
                    goal_angle_left = i
                    AVOID_FLAG = True
                    compute_currentYAW = yaw
                    compute_currentX=currentX
                    compute_currentY=currentY
                    #print "Point on the left:" + str(goal_angle_left)
                    break
                    
                    free_iterator_left = 0
            else:
                free_iterator_left = 0
            
        for i in range(0, (len(data_range) / 2)):
            if data_range[(len(data_range) / 2) - i] == 0:
                free_iterator_right = free_iterator_right + 1
                # print free_iterator_right
                if free_iterator_right == 2 * (obstacle_range + obstacle_angle):
                    goal_angle_right = i
                    AVOID_FLAG = True
                    compute_currentYAW = yaw
                    compute_currentX=currentX
                    compute_currentY=currentY
                    #print "Point on the right:" + str(goal_angle_right)
                    break
                    
                    free_iterator_right = 0
            else:
                free_iterator_right = 0
    print "Point on the right:" + str(goal_angle_right)
    print "Point on the left:" + str(goal_angle_left)


                   
    if AVOID_FLAG == True:        
        if goal_angle_right <= goal_angle_left:
            if goal_angle_right != 0:
                #print "right"
                direction = 1
                goalYaw = computeYaw(direction, goal_angle_right, yaw)
                distanceToDrive_x = math.tan(goalYaw) * lp_offset
                distanceToDrive_y = lp_offset / math.cos(goalYaw)
                print "Point to reach on x:" + str(distanceToDrive_x + compute_currentX)
                print "Point to reach on y:" + str(distanceToDrive_y + compute_currentY)
                twistYaw(goalYaw, yaw)
                #print "CURRENT YAW:" + str(yaw)
                #print "GOAL_YAW:" + str(goalYaw)
                if goalYaw + yaw_offset > yaw or goalYaw - yaw_offset < yaw:
                    YAW_FLAG = 0
                if YAW_FLAG == 0:
                    twistYaw(goalYaw, yaw)
                if YAW_FLAG == 1:
                    driveToPoint(distanceToDrive_y + compute_currentY, currentY)
            else :
                #print "left"
                direction = 0
                goalYaw = computeYaw(direction, goal_angle_left, yaw)
                distanceToDrive_x = math.tan(goalYaw) * lp_offset
                distanceToDrive_y = lp_offset / math.cos(goalYaw)
                print "Point to reach on x:" + str(distanceToDrive_x + compute_currentX)
                print "Point to reach on y:" + str(distanceToDrive_y + compute_currentY)
                twistYaw(goalYaw, yaw)
                #print "CURRENT YAW:" + str(yaw)
                #print "GOAL_YAW" + str(goalYaw) 
                if goalYaw + yaw_offset > yaw or goalYaw - yaw_offset < yaw:
                    YAW_FLAG = 0
                if YAW_FLAG == 0:
                    twistYaw(goalYaw, yaw)
                if YAW_FLAG == 1:
                    driveToPoint(distanceToDrive_y + compute_currentY, currentY)
        if goal_angle_left < goal_angle_right:
            if goal_angle_left != 0:
                print "left"
                direction = 0
                goalYaw = computeYaw(direction, goal_angle_left, yaw)
                distanceToDrive_x = math.tan(goalYaw) * lp_offset
                distanceToDrive_y = lp_offset / math.cos(goalYaw)
                print "Point to reach on x:" + str(distanceToDrive_x + compute_currentX)
                print "Point to reach on y:" + str(distanceToDrive_y + compute_currentY)
                twistYaw(goalYaw, yaw)
                #print "CURRENT YAW:" + str(yaw)
                #print "GOAL_YAW" + str(goalYaw)   
                if goalYaw + yaw_offset > yaw or goalYaw - yaw_offset < yaw:
                    YAW_FLAG = 0
                if YAW_FLAG == 0:
                    twistYaw(goalYaw, yaw)
                if YAW_FLAG == 1:
                    driveToPoint(distanceToDrive_y + compute_currentY, currentY)
            else:
                #print "right"
                direction = 1
                goalYaw = computeYaw(direction, goal_angle_right, yaw)
                distanceToDrive_x = math.tan(goalYaw) * lp_offset
                distanceToDrive_y = lp_offset / math.cos(goalYaw)
                print "Point to reach on x:" + str(distanceToDrive_x + compute_currentX)
                print "Point to reach on y:" + str(distanceToDrive_y + compute_currentY)
                twistYaw(goalYaw, yaw)
                #print "CURRENT YAW:" + str(yaw)
                #print "GOAL_YAW" + str(goalYaw)
                if goalYaw + yaw_offset > yaw or goalYaw - yaw_offset < yaw:
                    YAW_FLAG = 0
                if YAW_FLAG == 0:
                    twistYaw(goalYaw, yaw)
                if YAW_FLAG == 1:
                    driveToPoint(distanceToDrive_y + compute_currentY, currentY)
    #if REACH_GOAL==True:
       # PLANNER_FLAG=0
  
def odometry_data(data):
    # using global variables    
    yaw = tf_QuatQartesian(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    goal_yaw = getGoalYaw(data.pose.pose.position.x, data.pose.pose.position.y, goal_point_x, goal_point_y)
    # print yaw
    if PLANNER_FLAG == 0:
        global_planner(yaw, goal_yaw, data.pose.pose.position.x, data.pose.pose.position.y)
    if PLANNER_FLAG == 1:
        local_planner(yaw, data.pose.pose.position.x, data.pose.pose.position.y)   
           
def listener():
    print str(prefix)
    rospy.init_node('planner', anonymous=True)  # Node initialization
    rospy.Subscriber("/pioneer1/scan", LaserScan, laser_data)  # LaserScan subscriber initialization
    rospy.Subscriber("/pioneer1/odom", Odometry, odometry_data)  # Odometry subscriber initialization
    
    rospy.spin()  # Subscribers are working until the node will be killed
    

if __name__ == '__main__':
    listener()
    
