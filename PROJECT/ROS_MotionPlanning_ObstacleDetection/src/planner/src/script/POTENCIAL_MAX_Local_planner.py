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
from docutils.nodes import sidebar
#from BUG2_Local_planner import obstacle_range

#defining the global variables
start = True

################# GOAL POINTS ######################
prefix="pioneer1"
goal_point_x = -2
goal_point_y = 40
####################################################



##################### OFFSETS ######################
yaw_offset = 0.1  # offset to the yaw angle in which we have to reduce error
lp_offset = 1.5  # this offset declare the range in whoch the global planner should be off 
####################################################

##################### RANGES #######################
obstacle_range = 60 
angleDiv = 3.14 / 1000
distanctanceIntoMax=1.5
####################################################

##################### FLAGS ########################
YAW_FLAG = 0
PLANNER_FLAG = 0  
####################################################
##################### DATA #########################
data_range=0
goal_point = 0 
mask=0
goal_point=0
compute_currentYAW = 0
compute_currentX = 0
compute_currentY = 0   
yawObtain=False
result_p=0
#####################################################
def driveStop():
    pub = rospy.Publisher(prefix+'/cmd_vel', Twist)
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
    pub = rospy.Publisher(prefix+'/cmd_vel', Twist)
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
    pub = rospy.Publisher(prefix+'/cmd_vel', Twist)
    # rospy.init_node('twistYaw')
    twist = Twist()
    
    twist.angular.z = 0.00 
            
    pub.publish(twist)
    
def driveToPoint(goalP, currentP):
    global start
    
    pub = rospy.Publisher(prefix+'/cmd_vel', Twist)
    twist = Twist()
    
    
    if start == True:
        
        for i in range(0, 60):
            if i < 30:
                twist.linear.x = 0.1
            else:
                twist.linear.x = 0.2
                start = False
        
    if goalP > currentP or goalP < currentP:
            twist.linear.x = 0.0
            
            if goalP - 0.02 > currentP or goalP + 0.02 < currentP:
                twist.linear.x = 0.1
                if goalP - 0.2 > currentP or goalP + 0.2 < currentP:
                    twist.linear.x = 0.2
                    if (goalP - 1.0 > currentP) or (goalP + 1.0 < currentP):
                        twist.linear.x = 0.2
    print "Goal point:"+str(goalP)
    print "Current point:"+str(currentP)                    
    pub.publish(twist)

def computeYaw(goalPoint):
    
    if goalPoint > len(data_range)/2-1 == 0:
        #print "should compute to left"
        goalYaw = compute_currentYAW + angleDiv*2 * (goalPoint-len(data_range)/2)
    else:
        #print "should compute to right"
        goalYaw = compute_currentYAW - angleDiv*2 * (len(data_range)/2-goalPoint)
    #print len(data_range)/2-goalPoint
    #print angleDiv * (len(data_range)/2-goalPoint)
    return goalYaw

def getGoalYaw(startX, startY, goalX, goalY):
    # compute the yaw between start point and global point
    goal_yaw = math.atan2((goalY - startY), (goalX - startX))
    return goal_yaw

def makeMask(data):
    global result_p
    global goal_point
    mask=list(data)
    dataValue=list(data)
    length=len(data)/10   
    STOP_FLAG=1    
    for i in range(0, len(data)):
        if i<length:
            mask[i]=1
            mask[len(data)-1-i]=1
        if i>=length and i<length*2:
            mask[i]=1.05
            mask[len(data)-1-i]=1.05
        if i>=length*2 and i<length*3:
            mask[i]=1.1
            mask[len(data)-1-i]=1.1
        if i>=length*3 and i<length*4:
            mask[i]=1.15
            mask[len(data)-1-i]=1.15
        if i>=length*4 and i<length*5:
            mask[i]=1.2
            mask[len(data)-1-i]=1.2
    for i in range(0, len(data)):
        dataValue[i]=data_range[i]*mask[i]  
    #print "DATA VALUE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    MAX=dataValue.index(max(dataValue))
    for i in range (0, len(data_range)-1):
        print "DATA RANGE:" + str(i)+ "___val:" +str(data_range[i])
        print "DATA VAULE:" + str(i)+ "___val:" +str(dataValue[i])
        if i<(len(data_range)-1)/2:
            if STOP_FLAG==1:
                print "I am in the loop:" +str(dataValue[MAX])
                if dataValue[249-i]==dataValue[MAX]:
                     result_p=249-i                 
                     STOP_FLAG=0 
                if dataValue[249+i]==dataValue[MAX]:
                     result_p=249+i                 
                     STOP_FLAG=0   
    print result_p
 
def global_planner(yaw, goal_yaw, current_x, current_y):
    global YAW_FLAG
    print "print global planner is working"  
   # print goal_point_x
   # print goal_point_y
    #print current_x
   # print current_y  
    if goal_yaw + yaw_offset > yaw or goal_yaw - yaw_offset < yaw:
        YAW_FLAG = 0
    if YAW_FLAG == 0:
        twistYaw(goal_yaw, yaw)
    if YAW_FLAG == 1:
        driveToPoint(goal_point_y, current_y)
def test(yaw, currentX, currentY):
    global YAW_FLAG
    global compute_currentYAW
    global compute_currentX
    global compute_currentY
    global yawObtain
    global goal_point
    global PLANNER_FLAG
    goalYaw=0
    makeMask(data_range) 
    goal_point=result_p
    print "Local planner is working"
    # Obtain the side
    if yawObtain == False:
        driveStop()
        compute_currentYAW=yaw
        compute_currentX=currentX
        compute_currentY=currentY
        
        yawObtain = True
def local_planner(yaw, currentX, currentY):
    global YAW_FLAG
    global compute_currentYAW
    global compute_currentX
    global compute_currentY
    global yawObtain
    global goal_point
    global PLANNER_FLAG
    goalYaw=0
    
    
    print "Local planner is working"
    # Obtain the side
    if yawObtain == False:
        driveStop()
        compute_currentYAW=yaw
        compute_currentX=currentX
        compute_currentY=currentY
        makeMask(data_range) 
        goal_point=result_p 
        yawObtain = True
    
    goalYaw=computeYaw(goal_point)
    if goalYaw + yaw_offset > yaw or goalYaw - yaw_offset < yaw:
        YAW_FLAG = 0
    if YAW_FLAG == 0:
        twistYaw(goalYaw, yaw)
        print "CUrrent YAW" + str(yaw)
    if YAW_FLAG == 1:
        if yaw >3.14/4 and yaw<3.14/2: # I
             driveToPoint(compute_currentY+distanctanceIntoMax, currentY)
             if(currentY>compute_currentY+distanctanceIntoMax-0.02 and currentY<compute_currentY+distanctanceIntoMax+0.02):
                 PLANNER_FLAG=0
                 yawObtain = False
        if yaw >0 and yaw<3.14/4: #VIII
             driveToPoint(compute_currentX+distanctanceIntoMax, currentX) 
             if(currentX>compute_currentX+distanctanceIntoMax-0.02 and currentX<compute_currentX+distanctanceIntoMax+0.02):
                 PLANNER_FLAG=0
                 yawObtain = False  
        if yaw >-3.14/2 and yaw<-3.14/4: #VI
             driveToPoint(compute_currentX+distanctanceIntoMax, currentX)
             if(currentX>compute_currentX+distanctanceIntoMax-0.02 and currentX<compute_currentX+distanctanceIntoMax+0.02):
                 PLANNER_FLAG=0
                 yawObtain = False
        if yaw <0 and yaw>-3.14/4: #VII
             driveToPoint(compute_currentY+distanctanceIntoMax, currentY)    
             if(currentY>compute_currentY+distanctanceIntoMax-0.02 and currentY<compute_currentY+distanctanceIntoMax+0.02):
                 PLANNER_FLAG=0
                 yawObtain = False
        if yaw >3.14/2 and yaw<(3.14/2)*3: #II
             driveToPoint(compute_currentY+distanctanceIntoMax, currentY)
             if(currentY>compute_currentY+distanctanceIntoMax-0.02 and currentY<compute_currentY+distanctanceIntoMax+0.02):
                 PLANNER_FLAG=0
                 yawObtain = False
        if yaw >(3.14/2)*3 and yaw<3.14: #III
             driveToPoint(compute_currentX+distanctanceIntoMax, currentX) 
             if(currentX>compute_currentX+distanctanceIntoMax-0.02 and currentX<compute_currentX+distanctanceIntoMax+0.02):
                 PLANNER_FLAG=0
                 yawObtain = False  
        if yaw >-3.14 and yaw<(-3.14/2)*3: #IV
             driveToPoint(compute_currentX+distanctanceIntoMax, currentX)
             if(currentX>compute_currentX+distanctanceIntoMax-0.02 and currentX<compute_currentX+distanctanceIntoMax+0.02):
                 PLANNER_FLAG=0
                 yawObtain = False
        if yaw >(-3.14/2)*3 and yaw<-3.14/2: #V
             driveToPoint(compute_currentY+distanctanceIntoMax, currentY)    
             if(currentY>compute_currentY+distanctanceIntoMax-0.02 and currentY<compute_currentY+distanctanceIntoMax+0.02):
                 PLANNER_FLAG=0
                 yawObtain = False
                 
    
    print "PLANNER FLAG:" +str(PLANNER_FLAG)
    
def laser_data(data):
    global PLANNER_FLAG
    global data_range
    global goal_point
    if PLANNER_FLAG==0:
        data_range = list(data.ranges)  # get laser data in frame
        for i in range(0, len(data.ranges)):
            if data_range[i] < lp_offset:
                data_range[i]=0
                if PLANNER_FLAG == 0:
                    if i >= ((len(data.ranges) / 2 - 1) - obstacle_range ) and i <= ((len(data.ranges) / 2 - 1) + obstacle_range ):
                        for x in range(1, obstacle_range*2):
                            h=i-x                       
                            if i-x<0:
                                h=0
                            data_range[h]=0
                        for x in range(1, (obstacle_range)*3):
                            z=i+x                     
                            if i+x>len(data.ranges)-1: 
                                z=len(data.ranges)-1                       
                            data_range[z]=0                        
                        PLANNER_FLAG = 1          
                        print "LASE DATA:"+str (data_range)
                    
      
                  
def odometry_data(data):
    global compute_currentYAW
    # using global variables    
    yaw = tf_QuatQartesian(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    goal_yaw = getGoalYaw(data.pose.pose.position.x, data.pose.pose.position.y, goal_point_x, goal_point_y)
    # print yaw
    if PLANNER_FLAG == 0:
        global_planner(yaw, goal_yaw, data.pose.pose.position.x, data.pose.pose.position.y)
        #compute_currentYAW=yaw
    if PLANNER_FLAG == 1:
        #driveStop()
        #print data_range
        #driveStop()
        #test(yaw, data.pose.pose.position.x, data.pose.pose.position.y) 
        local_planner(yaw, data.pose.pose.position.x, data.pose.pose.position.y) 
    if PLANNER_FLAG == 2:
        driveStop()  
           
def listener():
    
    rospy.init_node('planner', anonymous=True)  # Node initialization
    rospy.Subscriber("/"+prefix+"/scan", LaserScan, laser_data)  # LaserScan subscriber initialization
    rospy.Subscriber("/"+prefix+"/odom", Odometry, odometry_data)  # Odometry subscriber initialization
    
    rospy.spin()  # Subscribers are working until the node will be killed
    

if __name__ == '__main__':
    listener()
    
