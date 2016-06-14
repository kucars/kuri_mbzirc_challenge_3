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
    rospy.init_node('follow_path')
    client = actionlib.SimpleActionClient('path_follower', FollowPathAction)
    client.wait_for_server()

    goal = FollowPathGoal()
    
    navTask = NavTask()
    obj = Object()
    objs = Objects()
    obj.width  = 100
    obj.height = 200
    obj.pose.pose.position = gm.Point(0,0,10.0)
    objs.objects.append(obj)
    task  = Task()
    task.uav_id   = 1
    task.uav_name = 'UAV1'
    task.object = obj

    path  = nav_msgs.Path()
    pose  = gm.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'
    pose.pose.position = gm.Point(25,65,10)
    pose.pose.orientation = gm.Quaternion(*quaternion_from_euler(0, 0, 0))
    path.poses.append(pose)
    
    navTask.path = path
    navTask.task = task    
    goal.navigation_task = navTask
    
    client.send_goal(goal)
    client.wait_for_result() 
    #client.wait_for_result(rospy.Duration.from_sec(5.0)) 
    print "Action Server Finished, with result:",client.get_result()
