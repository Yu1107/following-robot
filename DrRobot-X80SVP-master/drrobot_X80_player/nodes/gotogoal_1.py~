#!/usr/bin/env python

""" odom_out_and_back.py - Version 1.1 2013-12-20
1. https://www.mathworks.com/help/robotics/examples/path-following-for-differential-drive-robot.html;jsessionid=45afac83d5c230a3f8f9fa68da78
2. https://github.com/pirobot/rbx1/blob/indigo-devel/rbx1_nav/nodes/odom_out_and_back.py
3. http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal
4. https://github.com/wsustcid/openni2_tracker/blob/master/nodes/skeleton_follower.py
5. https://github.com/merose/diff_drive/blob/master/nodes/diff_drive_go_to_goal
6. https://www.coursera.org/lecture/mobile-robot/go-to-goal-wVmSR
7. https://www.youtube.com/watch?v=4Y7zG48uHRo
8. https://www.youtube.com/watch?v=D7ISrmszozk&t=4s
9. https://www.youtube.com/watch?v=4Y7zG48uHRo

    A basic demo of using the /odom topic to move a robot a given distance
    or rotate through a given angle.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion#, PoseStamped
import tf
from transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi, atan2,degrees
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry


global error_pre,I,D,thetai,thetad,thetad_p
error_pre = 0


I = 0
D = 0

thetad_p=0
thetai = 0
global shortok,imuok
shortok = False
imuok = False
global robot_position,rotation


class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('go_to_goal_1', anonymous=True)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

	msg = Bool()
	shortmsg = Bool()
        # Publisher the robot's is achieved
        #self.goal_achieved_pub = rospy.Publisher('/goal_achieved', Bool, queue_size=1)
	self.imu_subscriber = rospy.Subscriber('start_IMU', Bool,self.callback) 
        self.short_subscriber = rospy.Subscriber('short_kinect', Bool,self.shortback) 
	#self.odom_subscriber = rospy.Subscriber('odom', Odometry,self.get_odom) 

        # The maximum rotation speed in radians per second
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.2)
        
        # The minimum rotation speed in radians per second
        self.min_angular_speed = rospy.get_param("~min_angular_speed", 0.1)
        
        # The max linear speed in meters per second
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 1.0)
        
        # The minimum linear speed in meters per second
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0.1)

        self.distance_tolerance = rospy.get_param("~distance_tolerance",0.2) # meters
        
	self.kp = rospy.get_param("~kp",0.5)
	self.ki = rospy.get_param("~ki",0.05)
	self.kd = rospy.get_param("~kd",0.0)

	self.kpt = rospy.get_param("~kpi",4)
	self.kit = rospy.get_param("~kit",0.005)
	self.kdt = rospy.get_param("~kdt",0.0)

        # How fast will we update the robot's movement?
        rate = 20

        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)        
         
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(0.5)
   
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  
        	
      	
################### Wait for the  Pos topic to become available ##########################
	
	goal_topic = 'PDR_position' 
 	self.position_subscriber = rospy.Subscriber(goal_topic, Point, self.set_cmd_vel, queue_size=1)
	rospy.loginfo("Subscribing to PDR position...")  
      
    	# Wait for the  topic to become available
    	rospy.wait_for_message(goal_topic, Point)

	#self.goal = Point()
############################# Setup velcity and omega #######################################
    def distanceToGoal(self,goal,robot_position):
	#print robot_position.x,robot_position.y
	return sqrt(pow((goal.x - robot_position.x), 2) + pow((goal.y - robot_position.y), 2))

    def linear_vel(self, goal):
	return self.kp * self.distanceToGoal(goal,robot_position) #+ self.ki*I #+ self.kd*D 
 
    def angleToGoal(self,goal,rotation):
	#print rotation
	return (atan2(goal.y - robot_position.y, goal.x - robot_position.x) - rotation)

    def angular_vel(self, goal):
        return self.kpt * self.angleToGoal(goal,rotation) #+ self.kit*thetai #+ self.kdt*thetad

############################# Setup Bool Data #######################################


    def shortback(self,msg):
	global shortok
	shortok = msg.data 

    def callback(self,imumsg):
	global imuok
	imuok = imumsg.data

############################# Move to goal

     #Callback function implementing the "goal_position" value received
    def set_cmd_vel(self, position):
	goal = position
	#rospy.loginfo(" now -> goal: [%5.2f, %5.2f]", position.x,position.y)
        goal.x = round(position.x,4)
	goal.y = round(position.y,4)

	# Initialize the command
        move_cmd = Twist()

    	# Initialize the robot_position variable as a Point type
        robot_position = Point() 

	global robot_position,rotation
        # Get the starting position values
        (robot_position, rotation) = self.get_odom()	

	rospy.loginfo(" goal[%5.3f, %5.3f],robot[%5.3f, %5.3f]", goal.x,goal.y,robot_position.x,robot_position.y)
        


	global error_pre,I,D,thetai,thetad,thetad_p
	#distanceToGoal = sqrt(pow((goal.x - robot_position.x), 2) + pow((goal.y - robot_position.y), 2))
	#angleToGoal = (atan2(goal.y - robot_position.y, goal.x - robot_position.x) - rotation)

	while (self.distanceToGoal(goal,robot_position) >= self.distance_tolerance) :#and not rospy.is_shutdown():
		#rospy.loginfo(" now -> %s", shortok)   
		#D= self.distanceToGoal(goal,robot_position) - error_pre
		#I += self.distanceToGoal(goal,robot_position)
		#thetai += self.angleToGoal
		#thetad = self.angleToGoal - thetad_p
		#rospy.loginfo("v:[I(%5.2f),D(%5.2f)],w:[I(%5.2f),D(%5.2f)]",I,D,thetai,thetad)
		if (shortok == True or imuok == True):
			#if (self.distanceToGoal(goal,robot_position) > 1.0):	
				#Porportional Controller
				#linear velocity in the x-axis: constant = 1.5
				move_cmd.linear.x =  self.linear_vel(goal)
				move_cmd.linear.y = 0
				move_cmd.linear.z = 0			    
				#rospy.loginfo("x %5.2f y %5.2f error %5.2f",robot_position.x,robot_position.y,distanceToGoal);
		    		# Make sure we meet our min/max specifications
		   		move_cmd.linear.x = copysign(max(self.min_linear_speed, min(self.max_linear_speed, abs(move_cmd.linear.x))), move_cmd.linear.x)

				#angular velocity in the z-axis: constant = 4
				move_cmd.angular.x = 0
				move_cmd.angular.y = 0
				move_cmd.angular.z = self.angular_vel(goal)
				#rospy.loginfo("sendind %5.2f THETA %5.2f",rotation,angleToGoal)
				# Make sure we meet our min/max specifications
				move_cmd.angular.z = copysign(max(self.min_angular_speed, min(self.max_angular_speed, abs(move_cmd.angular.z))), move_cmd.angular.z)

				#Publishing our move_cmd
				self.cmd_vel.publish(move_cmd)

				# Publish at the desired rate.
				rospy.sleep(0.1)

				# Get the current position
				(robot_position, rotation) = self.get_odom()
				

				#rospy.loginfo(" distanceToGoal:[%5.2f], now -> cmd:[%5.2f,%5.2f], robot:[%5.2f,%5.2f]",distanceToGoal,move_cmd.linear.x,move_cmd.angular.z,robot_position.x,robot_position.y);
				rospy.loginfo(" ToGoal[%5.2f,%5.2f], robot[%5.3f,%5.3f]",self.distanceToGoal(goal,robot_position),self.angleToGoal(goal,rotation),robot_position.x,robot_position.y)
				#rospy.loginfo(" now -> robot:[%5.3f,%5.3f]",robot_position.x,robot_position.y)
				
				# Re-compute the distance,omega to the goal
				#distanceToGoal = sqrt(pow((goal.x - robot_position.x), 2) + pow((goal.y - robot_position.y), 2))
				#angleToGoal = (atan2(goal.y - robot_position.y, goal.x - robot_position.x) - rotation)
				#error_pre = self.distanceToGoal(goal,robot_position)
				#thetad_p = self.angleToGoal(goal,rotation)			

		elif(shortok == False):    
			rospy.loginfo("distance < 4m")
			break

	move_cmd.linear.x = 0
	move_cmd.angular.z = 0		
	self.cmd_vel.publish(move_cmd)

	#Stopping our robot after the movement is over
	rospy.loginfo(" Goal achieved.")



#########################################################################################################

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))
  
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
 	rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        OutAndBack()
	rospy.spin()# If we press control + C, the node will stop.
    except:
        rospy.loginfo("Out-and-Back node terminated.")

