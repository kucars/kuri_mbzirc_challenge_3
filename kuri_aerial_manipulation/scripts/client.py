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
    print "Start"
    client = actionlib.SimpleActionClient('aerialAction', PickObjectAction)
    client.wait_for_server()
    print "Waiting for server"
    goal = PickObjectGoal()
    obj = Object()
    obj.color  = "blue"
    goal.object_2_pick = obj
    client.send_goal(goal)
    print "Waiting for result"
    client.wait_for_result() 
    #client.wait_for_result(rospy.Duration.from_sec(5.0)) 
    print "Action Server Finished, with result:",client.get_result()
