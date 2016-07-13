#! /usr/bin/env python
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
     self.server.start()

   def execute(self, goal):
     print 'Mapping with UAV'
     success = True
     # start executing the action
     if self.server.is_preempt_requested():
         rospy.loginfo('MappingAction: Preempted')
         self.server.set_preempted()
         success = False

     self._feedback.objects_map = ObjectsMap()
     # publish the feedback
     self.server.publish_feedback(self._feedback)
      
     if success:
         rospy.loginfo('MappingAction: Succeeded')
         self.server.set_succeeded()
     else:
         rospy.loginfo('MappingAction: Failed')
     

