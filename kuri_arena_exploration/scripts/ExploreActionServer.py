#! /usr/bin/env python
import roslib
import rospy
import actionlib
from kuri_msgs.msg import *


class ExploreServer:

   def __init__(self):
     print 'Starting ExploreServer'
     self.server = actionlib.SimpleActionServer('ExploreAction', ExploreAction, self.execute, False)
     self.objectsmap = ObjectsMap()
     self._feedback = ExploreFeedback()
     self._result   = ExploreResult()
     self.server.start()

   def execute(self, goal):
     print 'Exploring with UAV ', goal.uav_id
     success = True
     # start executing the action
     if self.server.is_preempt_requested():
         rospy.loginfo('ExploreAction: Preempted')
         self.server.set_preempted()
         success = False

     self._feedback.area_percent_complete = 10.0
     # publish the feedback
     self.server.publish_feedback(self._feedback)
      
     if success:
         self._result.objects_map = self.objectsmap
         rospy.loginfo('ExploreAction: Succeeded')
         self.server.set_succeeded(self._result)
     else:
         rospy.loginfo('ExploreAction: Failed')
     

