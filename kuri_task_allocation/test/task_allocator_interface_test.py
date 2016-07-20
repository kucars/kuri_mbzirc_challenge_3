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

testName = 'test_objects_allocator'


class ObjectsAllocated(unittest.TestCase):
    def __init__(self, *args):
        super(ObjectsAllocated, self).__init__(*args)
        rospy.init_node(testName)

    # to run before each test
    # def setUp(self):
    #    self.objectsReceived = False

    # to run after each test
    # def tearDown():

    def test_object_allocation(self):

        client = actionlib.SimpleActionClient('actionAllocationServ', kuri_msgs.msg.AllocateTasksAction)
        #rospy.loginfo("Waiting for server")
        client.wait_for_server()
        #rospy.loginfo("setting up objects")

        goal = AllocateTasksGoal()

        obj = Object()
        obj.width = 100
        obj.height = 200
        obj.pose.pose.position.x = 400
        obj.pose.pose.position.y = 800
        obj.pose.pose.position.z = 100
        obj.color = "R"
        # obj.pose.pose.position = gm.Point(0, 0, 10.0)

        obj2 = Object()
        obj2.width = 100
        obj2.height = 200
        obj2.pose.pose.position.x = 1000
        obj2.pose.pose.position.y = 1000
        obj2.pose.pose.position.z = 100
        obj2.color = "R"
        #    obj2.pose.pose.position = gm.Point(0, 0, 10.0)

        obj3 = Object()
        obj3.width = 100
        obj3.height = 200
        obj3.pose.pose.position.x = 80
        obj3.pose.pose.position.y = 80
        obj3.pose.pose.position.z = 100
        obj3.color = "B"
        obj3.velocity.linear.x = 3
        obj3.velocity.linear.y = 4
        obj3.velocity.linear.z = 0

        # obj3.pose.pose.position = gm.Point(0, 0, 10.0)

        obj4 = Object()
        obj4.width = 100
        obj4.height = 200
        obj4.pose.pose.position.x = 900
        obj4.pose.pose.position.y = 150
        obj4.pose.pose.position.z = 100
        obj4.color = "B"


        obj6 = Object()
        obj6.width = 100
        obj6.height = 200
        obj6.pose.pose.position.x = 790
        obj6.pose.pose.position.y = 140
        obj6.pose.pose.position.z = 100
        obj6.color = "B"
        obj6.velocity.linear.x = 3
        obj6.velocity.linear.y = 4
        obj6.velocity.linear.z = 0


        # obj4.pose.pose.position = gm.Point(0, 0, 10.0)

        obj5 = Object()
        obj5.width = 100
        obj5.height = 200
        obj5.pose.pose.position.x = 600
        obj5.pose.pose.position.y = 200
        obj5.pose.pose.position.z = 100
        obj5.color = "B"


        objs = Objects()
        objs.objects.append(obj)
        objs.objects.append(obj2)
        objs.objects.append(obj3)
        objs.objects.append(obj4)
        #objs.objects.append(obj5)

        Map = ObjectsMap()

        Map.objects.append(obj)
        Map.objects.append(obj2)
        Map.objects.append(obj3)
        Map.objects.append(obj4)
        Map.objects.append(obj5)
        Map.objects.append(obj6)

        goal.objects_map = Map

        #rospy.loginfo("sending goal")
        client.send_goal(goal)
        rospy.loginfo("Waiting for result")
        client.wait_for_result()
        # client.wait_for_result(rospy.Duration.from_sec(5.0))
        #rospy.loginfo("Action Server Finished, with result:", 
        a = client.get_result()


        #
        #   sub = rospy.Subscriber('kuri_msgs/ObjectsMap', ObjectsMap, self.callback)

        #   while not self.objectsReceived:
        # rospy.loginfo("Waiting for objects")
        #     rospy.sleep(1)

        #   assert(self.objectsReceived)

        # def callback(self, msg):
        #   rospy.loginfo("Objects Received (%s)", len(msg.objects))
        #   if len(msg.objects)>=4:
        #       self.objectsReceived = True


if __name__ == '__main__':
    time.sleep(0.75)
    try:
        rostest.run('test_object_allocating', testName, ObjectsAllocated, sys.argv)
    except KeyboardInterrupt:
        pass
    print("exiting") 
