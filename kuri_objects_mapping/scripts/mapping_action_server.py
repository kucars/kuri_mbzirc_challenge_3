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

import roslib
import rospy
import actionlib
from kuri_msgs.msg import *



class MappingServer:

   def __init__(self):
     print 'Starting MappingServer'
     self.server = actionlib.SimpleActionServer('MappingAction', MappingAction, self.execute, False)
     self.objects_map = ObjectsMap()
     self._feedback = MappingFeedback()
     self._result   = MappingResult()
     self.server.start()
     self.hasGoal = False

   def execute(self, goal):
     print 'Mapping with UAV'
     success = True
     self.hasGoal = True
     # start executing the action
     if self.server.is_preempt_requested():
         rospy.loginfo('MappingAction: Preempted')
         self.server.set_preempted()
         success = False
         return
     print self.objects_map
     self._feedback.total_mapped_objects = len(self.objects_map.objects)
     # publish the feedback
     self.server.publish_feedback(self._feedback)
      
     if success:
         self._result.objects_map = ObjectsMap()
         self._result.objects_map = self.objects_map
         rospy.loginfo('MappingAction: Succeeded')
         self.server.set_succeeded(self._result)
     else:
         rospy.loginfo('MappingAction: Failed')
     
   def update(self, objects):
       self.objects_map = objects
       self._feedback.total_mapped_objects = len(self.objects_map.objects)
       self.server.publish_feedback(self._feedback)
       rospy.loginfo('MappingAction: Sending Objects Feedback')
