#!/usr/bin/env python
#
# Copyright 2016 KURI KUSTAR .
# Author Tarek Taha : tarek.taha@kustar.ac.ae
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

from __future__ import print_function

import sys
import time
import unittest

import rospy
import rostest
from std_msgs.msg import String
from kuri_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
testName = 'test_objects_landing'
velocityX = 0.0  
velocityY = 0.0 
velocityZ = 0.0 
# Position of the objetc "Currently looking for a way to make it dynamic or subscribe for it" 
objx = -2.961629
objy = 3.037924

class ObjectsLanded(unittest.TestCase):
    def __init__(self, *args):
        super(ObjectsLanded, self).__init__(*args)
        rospy.init_node(testName)
    
    # to run before each test
    def setUp(self):
        self.objectLanded = False
    
    #to run after each test
    #def tearDown():
    
    def test_objects_detection(self):
        #
        sub = rospy.Subscriber('/uav_1/mavros/local_position/odom', Odometry, self.callback)
        sub = rospy.Subscriber('/uav_1/mavros/setpoint_velocity/cmd_vel', TwistStamped, self.velocityCallback)
	


        while not self.objectLanded:
	  rospy.loginfo("Waiting for Landing")
          rospy.sleep(1)
		
	
        assert(self.objectLanded)



    def velocityCallback(self, msg):
	velocityX = msg.twist.linear.x 
	rospy.loginfo("velocityX " ,velocityX  )
	velocityY = msg.twist.linear.y 
	rospy.loginfo("velocityY " ,velocityY  )
	velocityZ = msg.twist.linear.z 
	rospy.loginfo("velocityZ " ,velocityZ  )


    def callback(self, msg):
        rospy.loginfo("Position Received (%s)", len(msg.objects))
	#print 'distance'
	#print sqrt( (msg.pose.pose.position.x - objx)^2  + (msg.pose.pose.position.y - objy)^2 )   )

	# condition that it landed in accurate way         
	if sqrt( (msg.pose.pose.position.x - objx)^2  + (msg.pose.pose.position.y - objy)^2 ) <= 0.02 and velocityX == 0 and velocityY== 0 and velocityZ==0 :
            self.objectLanded = True  
		 

if __name__ == '__main__':
    time.sleep(0.75)
    try:
        rostest.run('test_aerial_mani', testName, ObjectsLanded, sys.argv)
    except KeyboardInterrupt:
        pass
    print("exiting") 
