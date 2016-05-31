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

testName = 'test_objects_detection'


class ObjectsDetected(unittest.TestCase):
    def __init__(self, *args):
        super(ObjectsDetected, self).__init__(*args)
        rospy.init_node(testName)
    
    # to run before each test
    def setUp(self):
        self.objectsReceived = False
    
    #to run after each test
    #def tearDown():
    
    def test_objects_detection(self):
        #
        sub = rospy.Subscriber('kuri_msgs/ObjectsMap', ObjectsMap, self.callback)

        while not self.objectsReceived:
	  rospy.loginfo("Waiting for objects")
          rospy.sleep(1)

        assert(self.objectsReceived)

    def callback(self, msg):
        rospy.loginfo("Objects Received (%s)", len(msg.objects))
        if len(msg.objects)>=4:
            self.objectsReceived = True

if __name__ == '__main__':
    time.sleep(0.75)
    try:
        rostest.run('test_objects_detection', testName, ObjectsDetected, sys.argv)
    except KeyboardInterrupt:
        pass
    print("exiting") 
