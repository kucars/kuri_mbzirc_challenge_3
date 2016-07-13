#! /usr/bin/env python
import roslib
import rospy
import actionlib
from kuri_msgs.msg import *


class TrackingServer:

   def __init__(self):
     print 'Starting TrackingServer'
     self.server = actionlib.SimpleActionServer('TrackingAction', TrackingAction, self.execute, False)
     self.objects = Objects()
     self._feedback = MappingFeedback()
     self.server.start()

   def execute(self, goal):
     print 'Tracking with UAV'
     success = True
     # start executing the action
     if self.server.is_preempt_requested():
         rospy.loginfo('TrackingAction: Preempted')
         self.server.set_preempted()
         success = False

     self._feedback.objects = self.objects
     # publish the feedback
     self.server.publish_feedback(self._feedback)
      
     if success:
         rospy.loginfo('TrackingAction: Succeeded')
         self.server.set_succeeded()
     else:
         rospy.loginfo('TrackingAction: Failed')
     

