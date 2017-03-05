#! /usr/bin/env python
#Copyright (c) 2016, Buti Al Delail
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#* Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#* Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#* Neither the name of kuri_mbzirc_challenge_3 nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import thread
import threading
import time
import mavros
import actionlib
from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import *
from kuri_msgs.msg import *
from mapping_action_server import MappingServer

class ObjectsMapping:

    def __init__(self):
        self.objects = []
        self.objects_map = ObjectsMap()

        client = actionlib.SimpleActionClient('TrackingAction', TrackingAction)
        print "Waiting for tracker server"
        client.wait_for_server()        
        goal = TrackingGoal()
        goal.uav_id = 3
        client.send_goal(goal)
        print "Waiting for result"
        client.wait_for_result() 
        print "Result:",client.get_result()        
        
        self.sub = rospy.Subscriber("TrackingAction/feedback",Object, self.callback)

    def callback(self, actionServer):
        print 'Mapping: Recieving Tracked Objects --deprecated', actionServer
        #for obj in objects.feedback.tracked_objects.objects:
            ## TODO: Check and process objects
        #    self.objects.append(obj)
	#self.objects.append(actionServer.feedback.new_object)
        #self.objects_map.objects = self.objects;
        #self.objects_map.map = OccupancyGrid()
        
#        if self.actionServer.hasGoal:
#            self.actionServer.update(self.objects_map)
#        else:
#            self.actionServer.objects_map = self.objects_map
        
