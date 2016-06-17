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
    rospy.init_node('client')
    client = actionlib.SimpleActionClient('actionAllocationServ', kuri_msgs.msg.AllocateTasksAction)
    client.wait_for_server()

    goal = AllocateTasksGoal()

    obj = Object()
    obj.width = 100
    obj.height = 200
    obj.pose.pose.position.x = 450
    obj.pose.pose.position.y = 800
    obj.pose.pose.position.z = 100
    obj.color = "R"
    #obj.pose.pose.position = gm.Point(0, 0, 10.0)

    obj2 = Object()
    obj2.width = 100
    obj2.height = 200
    obj2.pose.pose.position.x = 1000
    obj2.pose.pose.position.y = 1000
    obj2.pose.pose.position.z = 100
    obj2.color = "R"
#    obj2.pose.pose.position = gm.Point(0, 0, 10.0)

    obj3 = Object()
    obj3.width = 100
    obj3.height = 200
    obj3.pose.pose.position.x = 80
    obj3.pose.pose.position.y = 80
    obj3.pose.pose.position.z = 80
    obj3.color = "R"

    #obj3.pose.pose.position = gm.Point(0, 0, 10.0)

    obj4 = Object()

    obj4.width = 100
    obj4.height = 200
    obj4.pose.pose.position.x = 800
    obj4.pose.pose.position.y = 100
    obj4.pose.pose.position.z = 100
    obj4.color = "R"

    #obj4.pose.pose.position = gm.Point(0, 0, 10.0)




    objs = Objects()
    objs.objects.append(obj)
    objs.objects.append(obj2)
    objs.objects.append(obj3)
    objs.objects.append(obj4)


    Map = ObjectsMap()

    Map.objects.append(obj)
    Map.objects.append(obj2)
    Map.objects.append(obj3)
    Map.objects.append(obj4)


    goal.objects_map = Map

    client.send_goal(goal)
    print "test"
    client.wait_for_result()
    # client.wait_for_result(rospy.Duration.from_sec(5.0))
    print "Action Server Finished, with result:", client.get_result()