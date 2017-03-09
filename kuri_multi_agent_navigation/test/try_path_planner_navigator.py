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
    print ("test path planning")
    client = actionlib.SimpleActionClient('/pathplanner_action_server', GeneratePathsAction)
    rospy.loginfo("Waiting for server")
    client.wait_for_server()
    rospy.loginfo("setting goal")
    assert("setting goal");
    goal = GeneratePathsGoal()
    obj1 = Object()
    objs = Objects()
    obj1.width  = 100
    obj1.height = 200
    obj1.pose.pose.position = gm.Point(27,62,10.0)
    objs.objects.append(obj1)
    task1  = Task()
    task1.uav_id   = 1
    task1.uav_name = 'UAV1'
    task1.object = obj1
    obj2 = Object()
    obj2.width  = 100
    obj2.height = 200
    obj2.pose.pose.position = gm.Point(32,63,9.0)
    task2  = Task()
    task2.uav_id   = 2
    task2.uav_name = 'UAV2'
    task2.object = obj2
    obj3 = Object()
    obj3.width  = 100
    obj3.height = 200
    obj3.pose.pose.position = gm.Point(25,53,9.0)
    task3  = Task()
    task3.uav_id   = 3
    task3.uav_name = 'UAV3'
    task3.object = obj3
    tks = Tasks()
    tks.tasks.append(task1)
    tks.tasks.append(task2)
    tks.tasks.append(task3)
    goal.tasks = tks
    rospy.loginfo("sending goal")
    client.send_goal(goal)
    rospy.loginfo("Waiting for result")
    client.wait_for_result()
    #client.wait_for_result(rospy.Duration.from_sec(5.0))
    result = client.get_result()
    rospy.loginfo(result)
    nav_tasks = result.nav_tasks
    for nav_task in nav_tasks.nav_tasks:
        actionservername = 'uav_%d/navigation_action_server' %(nav_task.task.uav_id)
        client = actionlib.SimpleActionClient(actionservername, FollowPathAction)
        rospy.loginfo(nav_task.task.uav_id);
        rospy.loginfo("Waiting for server")
        client.wait_for_server()
        rospy.loginfo("setting goal")
        assert("setting goal");
        goal = FollowPathGoal()
        goal.navigation_task = nav_task
        rospy.loginfo("sending goal")
        client.send_goal(goal)
        rospy.loginfo("Waiting for result")
        #client.wait_for_result()
        #result = client.get_result()
        #rospy.loginfo(result)
    while (client.get_state()==0 or client.get_state()==1):
        rospy.loginfo("waiting for result, status:%d",client.get_state())
    result = client.get_result()
    rospy.loginfo(result)
