#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
import socket
import math
from sensor_msgs.msg import Imu, MagneticField



def strings2Floats(listString):
	out=[]
	for j in range(0, len(listString)-1):
		out.append( float(listString[j]))
	return out	

if __name__=="__main__":
	rospy.init_node('PhoneToPC')
	imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=1)
        mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=1)
        imuMsg = Imu()
        magMsg = MagneticField()

        settings = termios.tcgetattr(sys.stdin)
	timeout = 20  #timeout in seconds
	bufferSize=1024 #bytes
	packSeparator="#"
	go=True;	
			
	UDPSocket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)	# creates a socket for udp transmission
	UDPSocket.settimeout(timeout)					# sets a time limit for no connection
	serverAddress = ('', 5555) 					 #listens for data stream from port 5555 , feel free to change to any port you like
 	print('Listening on port ' + str(5555))
	UDPSocket.bind(serverAddress)					# binds that socket to the address and port
        print('data receiving..')
	while go:
		[data,attr] = UDPSocket.recvfrom(bufferSize)#continuesly receive from that port into buffer 
		datastream=(data.decode("utf-8"))         #decode the data
		try:
			packages = datastream.split(packSeparator)  #'''splits the data stream based on # why ?? because "hyper imu" transfers aensor values separated by ',' and ending with '#'     '''

			for pack in packages:
				try:
					pack = pack+","
					words =strings2Floats(pack.split(","))
 					#numSensors = int(math.floor(len(words)/3))
				        
					if len(words) > 2:
					   # Publish message
					   imuMsg.header.stamp= rospy.Time.now()
					   imuMsg.header.frame_id = 'base_link'
					   imuMsg.linear_acceleration.x = words[2] # tripe axis accelerator meter
					   imuMsg.linear_acceleration.y = words[3]
					   imuMsg.linear_acceleration.z = words[4]

					   imuMsg.angular_velocity.x = words[6] # gyro
					   imuMsg.angular_velocity.y = words[7]
					   imuMsg.angular_velocity.z = words[8]

					   magMsg.magnetic_field.x = words[10] # mag
					   magMsg.magnetic_field.y = words[11]
					   magMsg.magnetic_field.z = words[12]

					   imu_pub.publish(imuMsg)
					   mag_pub.publish(magMsg)
				except:
					pass			

		except:
			pass
	UDPSocket.close() # relese the port


