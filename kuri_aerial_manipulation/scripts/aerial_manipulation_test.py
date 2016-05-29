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

from kuri_msgs.msg import *
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as gm
import tf.transformations
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from math import *

print "Aerial Manipulation Test"

def sendObject():
    pub = rospy.Publisher('kuri_msgs/Object', Object, queue_size=1,latch=True)
    rospy.init_node('aerial_manipulation_send_object', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    obj = Object()
    obj.width  = 100
    obj.height = 200
    obj.pose.pose.position = gm.Point(0,0,10.0)
    obj.color  = 'blue'

    pub.publish(obj)
    print "Object Published"
    while not rospy.is_shutdown():
        #yaw_degrees = 0  # North
        #yaw = radians(yaw_degrees)
        #quaternion = quaternion_from_euler(0, 0, yaw)
        #msg.pose.orientation = SP.Quaternion(*quaternion)
        #rospy.loginfo(msg.pose)
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        sendObject()
    except rospy.ROSInterruptException:
        pass