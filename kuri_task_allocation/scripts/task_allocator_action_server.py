#!/usr/bin/env python



#
#
import math
import actionlib
import rospy
from std_msgs.msg import String
import random
import numpy
import yaml
# from matplotlib.pyplot import figure, show
import matplotlib.pyplot as plt
import ast
import geometry_msgs
from kuri_msgs.msg import *
from geometry_msgs.msg import Pose
from mavros import setpoint as SP
import rospkg
from task_allocator import callbackLogic


class actionAllocator(object):
    # create messages that are used to publish feedback/result
    _feedback = kuri_msgs.msg.AllocateTasksActionFeedback()
    _result = kuri_msgs.msg.AllocateTasksResult()

    locationOfUAVs = []

    def __init__(self, name):

        #rospy.Subscriber("nav_msgs/Odometry", ObjectsMap, callbackloc)  # I add the kuri-message type here


        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, kuri_msgs.msg.AllocateTasksAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()




    def callbackloc(data2):

        global locationOfUAVs
        locationOfUAVs = data2

    def execute_cb(self, goal):
        global locationOfUAVs
        data = goal.objects_map

        callback5 = callbackLogic(data)

        callbackreturns = callback5
        success = True
        # start executing the action

#        for i in xrange(1, goal.order):
#            # check that preempt has not been requested by the client
#            if self._as.is_preempt_requested():
#                rospy.loginfo('%s: Preempted' % self._action_name)
#                self._as.set_preempted()
#                success = False
#                break
#            self._feedback.sequence.append(callbackreturns)
#            # publish the feedback
#            self._as.publish_feedback(self._feedback)
#            # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
#            r.sleep()

        if success:
            self._result.tasks = callbackreturns
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)





if __name__ == '__main__':
    rospy.init_node("actionAllocationServ")
    actionAllocator(rospy.get_name())

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


