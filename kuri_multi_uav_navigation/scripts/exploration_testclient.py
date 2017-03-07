#! /usr/bin/env python
import roslib
import rospy
import actionlib
from kuri_msgs.msg import *
from std_msgs.msg import Int32
from nav_msgs.msg import *
from geometry_msgs.msg import *
import tf.transformations
from tf.transformations import quaternion_from_euler
import rospkg 
from actionlib_msgs.msg import GoalStatus
    

if __name__ == '__main__':
    rospy.init_node('ExplorerClient')
    print "Starting exploration Client Test"
    client = actionlib.SimpleActionClient('/uav_1/navigation_action_server1', FollowPathAction)
    client.wait_for_server()
    print "Waiting for server"
    goal = FollowPathGoal()
    
    rospack = rospkg.RosPack()
    theFile = open(rospack.get_path('kuri_system_coordinator')+"/config/exploration_waypoints_50.txt", "r")
    data = theFile.readlines()
    p = Path()
    for line in data:
	poses = line.split()
	pt = PoseStamped()
	pt.pose.position.x = float(poses[0])
	pt.pose.position.y = float(poses[1])
	pt.pose.position.z = float(poses[2])
	p.poses.append(pt)
	
    nav = NavTask()
    task = Task()
    task.uav_id = 1
    nav.path = p
    nav.task = task    
    goal.navigation_task = nav
    
    while(not rospy.is_shutdown()):
      client.send_goal(goal)
      print "Waiting for result"
      client.wait_for_result() 
      print "Result:",client.get_result()
      #if( not (client.get_result() == GoalStatus.SUCCEEDED) ):
	#break

