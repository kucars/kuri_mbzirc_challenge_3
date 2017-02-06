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
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
import smach
import smach_ros
from kuri_msgs.msg import GenerateExplorationWaypointsAction, Task, NavTask,NavTasks, FollowPathAction,TrackingAction
from smach_ros import SimpleActionState
from actionlib_msgs.msg import GoalStatus

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
	rospy.loginfo('Executing state generating waypoints\n\n')
	
    def goal_callback(self, userdata, goal):
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
	  
	  
class Exploring(smach_ros.SimpleActionState):
    """Explore the arena 
    Outcomes
    --------
        succeeded : moved to the waypoint
        preempted : a cancel request by the client occured
        aborted : an error occured in the navigation action server
    
    input_keys
    ----------
	exploring_in : the waypoint location (gps or local)
    """
    
    def __init__(self):
	smach_ros.SimpleActionState.__init__(self,'navigation_action_server',
						  FollowPathAction,
						  goal_slots=['navigation_task'],
						  result_cb=self.result_callback,						  
						  input_keys=['navigation_task']  
					    )
	rospy.loginfo('Executing state Exploring\n\n')

    def result_callback(self, userdata, status, result):
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
						    result_cb=self.result_callback,
						    output_keys=['detecting_objects_out']  
					    )
	rospy.loginfo('Executing state Detecting Objects\n\n')
	
    def goal_callback(self, userdata, goal):
	task = Task(uav_id=1)
        return task	
      
    def result_callback(self, userdata, status, result):
	if status == GoalStatus.SUCCEEDED:
	  userdata.detecting_objects_out = result.total_objects_tracked
          return 'succeeded'
	elif status == GoalStatus.PREEMPTED:
	  return 'preempted'
	else:
	  return 'aborted'	
