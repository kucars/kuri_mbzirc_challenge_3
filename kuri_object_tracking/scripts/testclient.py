#! /usr/bin/env python
import roslib
import rospy
import actionlib
from kuri_msgs.msg import *
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as gm
import tf.transformations
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('Client')
    print "Starting Tracking Client Test"
    client = actionlib.SimpleActionClient('TrackingAction', TrackingAction)
    client.wait_for_server()
    print "Waiting for server"
    goal = TrackingGoal()
    client.send_goal(goal)
    print "Waiting for result"
    client.wait_for_result() 
    print "Result:",client.get_result()
