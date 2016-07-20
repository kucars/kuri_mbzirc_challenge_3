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
                client = actionlib.SimpleActionClient('/pathplanner_action_server', GeneratePathsAction)
		rospy.loginfo("Waiting for server")
		client.wait_for_server()
		rospy.loginfo("setting goal")
                assert("setting goal");
		goal = GeneratePathsGoal()
                obj1 = Object()
		objs = Objects()
                obj1.width  = 100
                obj1.height = 200
                obj1.pose.pose.position = gm.Point(5,5,10.0)
                objs.objects.append(obj1)
                task1  = Task()
                task1.uav_id   = 1
                task1.uav_name = 'UAV1'
                task1.object = obj1
                obj2 = Object()
                obj2.width  = 100
                obj2.height = 200
                obj2.pose.pose.position = gm.Point(3,3,10.0)
                task2  = Task()
                task2.uav_id   = 2
                task2.uav_name = 'UAV2'
                task2.object = obj2
                obj3 = Object()
                obj3.width  = 100
                obj3.height = 200
                obj3.pose.pose.position = gm.Point(2,2,10.0)
                task3  = Task()
                task3.uav_id   = 3
                task3.uav_name = 'UAV3'
                task3.object = obj3
		tks = Tasks()
                tks.tasks.append(task1)
                tks.tasks.append(task2)
                tks.tasks.append(task3)
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
