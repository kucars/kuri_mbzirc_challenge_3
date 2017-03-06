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

import roslib
import rospy
import actionlib
from kuri_msgs.msg import *
from object_tracking import *

class TrackingServer:

   def __init__(self):
     print 'Starting TrackingServer'
     #self.newObjectPub = rospy.Publisher("TrackingAction/NewObject",Object, queue_size=10)
     self.server = actionlib.SimpleActionServer('TrackingAction', TrackingAction, self.execute, False)
     self._feedback = TrackingFeedback()
     self._result   = TrackingResult()
     self.tracking = None
     self.hasGoal = False
     self.server.start()


   def execute(self, goal):
     print 'Tracking with UAV'
     success = True
     self.hasGoal = True
     # start executing the action
     if self.server.is_preempt_requested():
         rospy.loginfo('TrackingAction: Preempted')
         self.server.set_preempted()
         success = False
         return
     if self.tracking == None:
	 self.tracking = object_tracking(self,goal)
     self._feedback.new_object = Object()
     #self._feedback.new_object = self.tracking.obstacles #self.total_objects
     # publish the feedback
     #self.server.publish_feedback(self._feedback)
      
     if success:
         self._result.tracked_objects = self.tracking.obstacles#self.total_objects
         rospy.loginfo('TrackingAction: Succeeded')
         self.server.set_succeeded(self._result)
     else:
         rospy.loginfo('TrackingAction: Failed')
     
   def update(self, new_object):
       print new_object
       self._feedback.new_object = new_object
       self.server.publish_feedback(self._feedback)
       rospy.loginfo('TrackingAction: Sending Objects Feedback')
       
   #def newObjectPublish(self, newObject):
   #    self.newObjectPub.publish()
   #    rospy.loginfo('TrackingAction: Sending New Object')

def main(args):
  rospy.init_node('mbzirc_challenge3_object_tracking', anonymous=True)
  action = TrackingServer()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
