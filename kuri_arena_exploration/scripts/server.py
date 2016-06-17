#! /usr/bin/env python
import roslib
import rospy
import actionlib
from kuri_msgs.msg import *


class ExploreServer:

   def __init__(self):
     self.server = actionlib.SimpleActionServer('Explore', ExploreAction, self.execute, False)
     self.server.start()
     self.objectsmap = ObjectsMap()

   def execute(self, goal):
     self.server.set_succeeded()
     self.objectsmap = goal.objects_map

 if __name__ == '__main__':
   rospy.init_node('explore_server')
   server = ExploreServer()
   rospy.spin()

