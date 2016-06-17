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
    rospy.init_node('generate_paths')
    client = actionlib.SimpleActionClient('/path_planning', GeneratePathsAction)
    print "waiting for server"
    client.wait_for_server()
    print "setting goal"
    goal = GeneratePathsGoal()
    
    obj = Object()
    objs = Objects()
    obj.width  = 100
    obj.height = 200
    obj.pose.pose.position = gm.Point(5,5,10.0)
    objs.objects.append(obj)
    task  = Task()
    task.uav_id   = 1
    task.uav_name = 'UAV1'
    task.object = obj
	
    tks = Tasks()
    tks.tasks.append(task)    
    goal.tasks = tks
    print "sending goal"
    client.send_goal(goal)
    print "waiting for result"
    client.wait_for_result() 
    #client.wait_for_result(rospy.Duration.from_sec(5.0)) 
    print "Action Server Finished, with result:",client.get_result()
