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
from exploration import *
from kuri_msgs.msg import *


class ExploreServer:

   def __init__(self):
     print 'Starting ExploreServer'
     self.server = actionlib.SimpleActionServer('ExploreAction', ExploreAction, self.execute, False)
     self.objects_map = ObjectsMap()
     self._feedback = ExploreFeedback()
     self._result   = ExploreResult()
     self.server.start()
     self.hasGoal = False
     self.exploration = Exploration()

   def execute(self, goal):
     print 'Exploring with UAV ', goal.uav_id
     success = True
     self.hasGoal = True
     # start executing the action
     if self.server.is_preempt_requested():
         rospy.loginfo('ExploreAction: Preempted')
         self.server.set_preempted()
         success = False
         return
     if self.exploration.isExploring == False:
         thread.start_new_thread(self.exploration.explore, ())
     self.exploration.client.wait_for_server()        
     goal = MappingGoal()
     goal.uav_id = 3
     self.exploration.client.send_goal(goal)
     print "Waiting for result"
     self.exploration.wait_for_result() 
     print "Result:"
     self.objects_map = self.exploration.client.get_result().objects_map
     print self.objects_map        
     
     self._feedback.area_percent_complete = 10.0
     # publish the feedback
     self.server.publish_feedback(self._feedback)
      
     if success:
         self._result.objects_map = self.objects_map
         rospy.loginfo('ExploreAction: Succeeded')
         self.server.set_succeeded(self._result)
     else:
         rospy.loginfo('ExploreAction: Failed')


   def update(self, objects, progress):
       self.objects_map = objects
       self._feedback.area_percent_complete = progress#len(self.objects_map.objects)
       self.server.publish_feedback(self._feedback)
       rospy.loginfo('ExploreAction: Sending Objects Feedback')

    
def main(args):
  rospy.init_node('exploration')
  #mavros.set_namespace()  # initialize mavros module with default namespace
  mavros.set_namespace('/uav_3/mavros')   
  actionserver = ExploreServer()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
 
