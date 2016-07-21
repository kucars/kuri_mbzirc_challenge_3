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

#! /usr/bin/env python


import roslib
import rospy
import actionlib
from kuri_msgs.msg import *
from std_msgs.msg import Int32
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as gm
import tf.transformations
from tf.transformations import quaternion_from_euler

def callback(obj):
    #print "Recieved Feedback: Tracker has "
    print obj
    #print " Tracked Objects"
    

if __name__ == '__main__':
    rospy.init_node('Client')
    print "Starting Explore Client Test"
    client = actionlib.SimpleActionClient('ExploreAction', ExploreAction)
    client.wait_for_server()
    sub = rospy.Subscriber("ExploreAction/feedback",Int32, callback)
    print "Waiting for server"
    goal = ExploreGoal()
    goal.uav_id = 3
    client.send_goal(goal)
    print "Waiting for result"
    client.wait_for_result() 
    print "Result:",client.get_result()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"