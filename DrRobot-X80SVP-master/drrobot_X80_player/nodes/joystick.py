#!/usr/bin/env python

#note on plain values:
#buttons are either 0 or 1
#button axes go from 0 to -1
#stick axes go from 0 to +/-1


#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

import rospy
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


rospy.init_node('joystick', anonymous=True)
r = rospy.Rate(10) #10hz

msg = Twist()
IMUmsg = Bool()
goalmsg = Point()
a_msg = Bool()

pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
IMU_pub = rospy.Publisher('initial_position', Bool, queue_size=1)
goal_pub = rospy.Publisher('/PDR_position', Point, queue_size=1)
get_a_pub = rospy.Publisher('/get_a', Bool, queue_size=1)

def callbackJoy(data):
    global msg,IMUmsg,a_msg,goalmsg
    #print (data.axes[1])
    #print (data.axes[0])

    if (data.axes[3] != 0) or (data.axes[2] != 0):
    	msg.linear.x = data.axes[3]*0.85
    	msg.angular.z = data.axes[2]*1.5
    	pub.publish(msg)
    	print (msg.linear.x,msg.angular.z)


    ## IMU initial_position
    if (data.buttons[13] == 1):
       IMUmsg.data = True
       IMU_pub.publish(IMUmsg)
    
    if (data.buttons[14] == 1): 
    	msg.linear.x = 0
    	msg.angular.z = 0
    	pub.publish(msg)

    # get skeleton leg length
    if (data.buttons[15] == 1): 
    	a_msg.data = True
    	get_a_pub.publish(a_msg)
	if (a_msg.data == True):
		print (a_msg.data)

    """if (data.buttons[12] == 1):
       goalmsg.x = 0.0
       goalmsg.y = 0.0

       goal_pub.publish(goalmsg)"""

rospy.Subscriber("joy", Joy, callbackJoy)
def talker():    
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

