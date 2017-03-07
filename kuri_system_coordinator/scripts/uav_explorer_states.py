#!/usr/bin/env python
#
#   Copyright (C) 2006 - 2016 by                                          
#      Tarek Taha, KURI  <tataha@tarektaha.com>                           
#      Randa Almadhoun, KURI <randa.almadhoun@gmail.com>

#   This program is free software; you can redistribute it and/or modify  
#   it under the terms of the GNU General Public License as published by  
#   the Free Software Foundation; either version 2 of the License, or     
#   (at your option) any later version.                                   
#                                                                         
#   This program is distributed in the hope that it will be useful,       
#   but WITHOUT ANY WARRANTY; without even the implied warranty of        
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         
#   GNU General Public License for more details.                          
#                                                                         
#   You should have received a copy of the GNU General Public License     
#   along with this program; if not, write to the                         
#   Free Software Foundation, Inc.,                                       
#   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          


import rospy
import thread
import threading
import time
import mavros
import actionlib

from math import *
from mavros.utils import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
import smach
import smach_ros
from kuri_msgs.msg import GenerateExplorationWaypointsAction, Task, NavTask,NavTasks, FollowPathAction,TrackingAction, MappingAction,Object,Objects
from smach_ros import SimpleActionState
from actionlib_msgs.msg import GoalStatus
import random
import rospkg 

class GenerateWaypoints(smach_ros.SimpleActionState):
    """generates a set of waypoints for covering the entrire arena 
    Outcomes
    --------
        succeeded : generate the set of waypoints
        preempted : a cancel request by the client occured
        aborted : an error occured in the exploration waypoints action server
   
    input_keys
    ----------
	generate_waypoints_in : the starting location (gps or local)
	
    output_keys
    ----------
	generate_waypoints_out : a set of waypoints the UAV should pass to entirely map the arena
    """
    
    def __init__(self):
	smach_ros.SimpleActionState.__init__(self,'exploration_waypoints_action_server',
						  GenerateExplorationWaypointsAction,
						  goal_cb=self.goal_callback,
						  result_cb=self.result_callback,
						  #TODO: the input key will be added later when using really the gps data (now I'm defining the start in the code)
						  input_keys=['generate_waypoints_in'], 
						  output_keys=['generate_waypoints_out']
					    )

    def goal_callback(self, userdata, goal):
	rospy.loginfo('Executing state generating waypoints\n\n')
	task = Task(uav_id=1)
        return task
      
    def result_callback(self, userdata, status, result):
	if status == GoalStatus.SUCCEEDED:
	  userdata.generate_waypoints_out = result.expPath
          return 'succeeded'
	elif status == GoalStatus.PREEMPTED:
	  return 'preempted'
	else:
	  return 'aborted'

class ReadWaypoints(smach.State):
    """generates a set of waypoints for covering the entrire arena 
    Outcomes
    --------
        succeeded : generate the set of waypoints
        preempted : a cancel request by the client occured
        aborted : an error occured in the exploration waypoints action server
   
    input_keys
    ----------
	generate_waypoints_in : the starting location (gps or local)
	
    output_keys
    ----------
	generate_waypoints_out : a set of waypoints the UAV should pass to entirely map the arena
    """
    
    def __init__(self):
        smach.State.__init__(self, 
			     outcomes=['succeeded','aborted'],
			     input_keys=['generate_waypoints_in'], 
			     output_keys=['generate_waypoints_out']
			     
			     )

    def execute(self, userdata):
	rospy.loginfo('Executing state reading waypoints\n\n')
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
	userdata.generate_waypoints_out = nav
	return 'succeeded'



class Mapping(smach_ros.SimpleActionState):
    """ perform mapping to the arena  
    Outcomes
    --------
        succeeded : mapping arena is done
        preempted :  map is preempted by client request
        aborted : an error occured in the mapping action server
	
    output_keys
    ----------	
	mapping_out : object map (map + objects) it should be published    
    """  
    def __init__(self):
	smach_ros.SimpleActionState.__init__(self,'create_map',
						  MappingAction,
						  goal_cb=self.goal_callback,
						  result_cb=self.result_callback
					    )  
    def goal_callback(self, userdata, goal):
        rospy.loginfo('Executing state Mapping\n\n')
	task = Task(uav_id=1)
        return task
      
    def result_callback(self, userdata, status, result):
	if status == GoalStatus.SUCCEEDED:
          return 'succeeded'
	elif status == GoalStatus.PREEMPTED:
	  return 'preempted'
	else:
	  return 'aborted'
	  
	  
class Exploring(smach_ros.SimpleActionState):
    """Explore the arena 
    Outcomes
    --------
        succeeded : moved to the waypoint
        preempted : a cancel request by the client occured
        aborted : an error occured in the navigation action server
    
    input_keys
    ----------
	navigation_task : the exploration path generated by waypoints generator for exploration
    """
    
    def __init__(self,actionServerNS):
	smach_ros.SimpleActionState.__init__(self,actionServerNS,
						  FollowPathAction,
						  goal_slots=['navigation_task'],
						  result_cb=self.result_callback,						  
						  input_keys=['navigation_task']  
					    )

    def result_callback(self, userdata, status, result):
      	rospy.loginfo('Executing state Exploring\n\n')
	if status == GoalStatus.SUCCEEDED:
          return 'succeeded'
	elif status == GoalStatus.PREEMPTED:
	  return 'preempted'
	else:
	  return 'aborted'
	
class DetectingObjects(smach_ros.SimpleActionState):
    """detecting the objects at the waypoints of the exploration  
    Outcomes
    --------
        succeeded : objects detection is done passing through the arena (exploration)
        preempted : a cancel request by the client occured
        aborted : an error occured in the tracker action server
	
    output_keys
    ----------
	detecting_objects_out : detected objects locations
    """  
    def __init__(self):
        smach_ros.SimpleActionState.__init__(self, 'TrackingAction',
						    TrackingAction,
						    goal_cb=self.goal_callback,
						    result_cb=self.result_callback
						   # output_keys=['detecting_objects_out']  
					    )
	
    def goal_callback(self, userdata, goal):
      	rospy.loginfo('Executing state Detecting Objects\n\n')
	task = Task(uav_id=1)
        return task	
      
    def result_callback(self, userdata, status, result):
	if status == GoalStatus.SUCCEEDED:
	  #userdata.detecting_objects_out = result.total_objects_tracked
          return 'succeeded'
	elif status == GoalStatus.PREEMPTED:
	  return 'preempted'
	else:
	  return 'aborted'

class DetectingObjectsS(smach.State):
    """detecting the objects at the waypoints of the exploration  
    Outcomes
    --------
        succeeded : objects detection is done passing through the arena (exploration)
        preempted : a cancel request by the client occured
        aborted : an error occured in the tracker action server
	
    """  
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['succeeded',
                                       'aborted',
                                       'preempted']
                             )
	self.detectedObjects = Objects()		     
	self.objects_pub = rospy.Publisher("/TrackingAction/NewObjects",Objects, queue_size=10)

    def execute(self, userdata):
      rospy.loginfo('Executing state Detecting Objects (example)\n\n')
      self.addObjects(2) #example of 2 red objects
      self.objects_pub.publish(self.detectedObjects)
      del self.detectedObjects.objects[:]
      rospy.sleep(10)
      return 'succeeded'
      return 'preempted'
      return 'aborted'
   
    def addObjects(self, numOfObjects):   
      i=0
      c=0
      while i<numOfObjects:
	pose = PoseWithCovariance()
	pose.pose.position.x = random.randrange(-45, 45, 2)
	pose.pose.position.y = random.randrange(-30, 30, 1)
	pose.pose.position.z = 10
	print("object %i: x:%f, y:%f, z:%f" % (i,pose.pose.position.x,pose.pose.position.y,pose.pose.position.z))
	newObject = Object()
	newObject.pose = pose
	newObject.velocity = Twist()
	newObject.width = 100
	newObject.height = 100
	if(c==0):
	    newObject.color = 'RED'
	    c=1
	elif(c==1):
	    newObject.color = 'GREEN'
	    c=2
	else:
	    newObject.color = 'BLUE'
	    c=0
	self.detectedObjects.objects.append(newObject)
	i=i+1

	
