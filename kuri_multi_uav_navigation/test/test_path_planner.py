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
import actionlib

import rospy
import rostest
from std_msgs.msg import String
from kuri_msgs.msg import *
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as gm

testName = 'test_path_planner'


class PathPlanGenerated(unittest.TestCase):
    def __init__(self, *args):
        super(PathPlanGenerated, self).__init__(*args)
        rospy.init_node(testName)
		
    # to run before each test
    #def setUp(self):
    #    self.objectsReceived = False
    
    #to run after each test
    #def tearDown():
    
    def test_path_planning(self):
		client = actionlib.SimpleActionClient('/path_planning', GeneratePathsAction)
		rospy.loginfo("Waiting for server")
		client.wait_for_server()
		rospy.loginfo("setting goal")
		goal = GeneratePathsGoal()
		obj = Object()
		objs = Objects()
		obj.width  = 100
		obj.height = 200
		obj.pose.pose.position = gm.Point(5,5,10.0)
		objs.objects.append(obj)
		task  = Task()
		task.uav_id   = 1
		task.uav_name = 'UAV1'
		task.object = obj
		
		tks = Tasks()
		tks.tasks.append(task)
		goal.tasks = tks
		rospy.loginfo("sending goal")
		client.send_goal(goal)
		rospy.loginfo("Waiting for result")
		client.wait_for_result() 
		#client.wait_for_result(rospy.Duration.from_sec(5.0)) 
		rospy.loginfo(client.get_result())
        #
     #   sub = rospy.Subscriber('kuri_msgs/ObjectsMap', ObjectsMap, self.callback)

     #   while not self.objectsReceived:
	 # rospy.loginfo("Waiting for objects")
     #     rospy.sleep(1)

     #   assert(self.objectsReceived)

    #def callback(self, msg):
     #   rospy.loginfo("Objects Received (%s)", len(msg.objects))
     #   if len(msg.objects)>=4:
     #       self.objectsReceived = True

if __name__ == '__main__':
    time.sleep(0.75)
    try:
        rostest.run('test_path_planning', testName, PathPlanGenerated, sys.argv)
    except KeyboardInterrupt:
        pass
    print("exiting") 
