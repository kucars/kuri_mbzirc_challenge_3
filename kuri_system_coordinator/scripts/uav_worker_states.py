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
import smach
import smach_ros
import actionlib

from math import *
from mavros.utils import *
from geometry_msgs.msg import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from kuri_msgs.msg import *
from kuri_msgs.msg import GenerateExplorationWaypointsAction, Task, NavTask,NavTasks, FollowPathAction,TrackingAction, GeneratePathsAction
from smach_ros import SimpleActionState
from actionlib_msgs.msg import GoalStatus




class WaitingForTask(smach.State):## unnecessary state (in my opinion)
    """ waiting for the tasks   
    Outcomes
    --------
        waiting : waiting for the tasks to be allocated 
        ready :  tasks are allocated
    
    input_keys
    ----------
	waiting_for_tasks_in : tasks that has been allocated
	
    output_keys
    ----------	
	waiting_for_tasks_out :    
    """  
    def __init__(self, sleep_t):
        smach.State.__init__(self, outcomes=['waiting','ready'],input_keys=['waiting_for_tasks_in'],output_keys=['waiting_for_tasks_out'])
        self.counter = 0
        self.sleep_t=sleep_t
        
    def execute(self, userdata):
        rospy.loginfo('Executing state WaitingForTask\n\n')
        #userdata.waiting_for_tasks_out=1
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            return 'waiting'
        else:
	    userdata.waiting_for_tasks_out = userdata.waiting_for_tasks_in + 1 #userdata example
            return 'ready'
	  
class NavTasksLoop(smach.State):
    """ looping through nav tasks returned by path genrator
    Outcomes
    --------
        looping : going to the hovering position of the object
        loopFinished : I executed all the paths for this worker
	
    input_keys
    ----------
	looping_in : the given tasks by the path generator (NavTasks)

    input_keys
    ----------
	looping_out : one of the nav tasks given to the uav (this will be passed to the uav Navigating2Object state)	
    """  
    def __init__(self):
	smach.State.__init__(self, outcomes=['loopFinished','looping'],input_keys=['looping_in'], output_keys=['looping_out'])
	self.counter = 0

    def execute(self, userdata):
      	rospy.loginfo('Executing state NavTasksLooping\n\n')
	
        if self.counter < len(userdata.looping_in.nav_tasks):
	    nav = NavTask()
      	    nav = userdata.looping_in.nav_tasks[self.counter]
      	    userdata.looping_out = nav
      	    print("uav %i \n\n" % (nav.task.uav_id))
      	    print("nav task %i is (%f,%f,%f) \n\n" % (self.counter,nav.path.poses[0].pose.position.x, nav.path.poses[0].pose.position.y,nav.path.poses[0].pose.position.z))
            self.counter += 1
            return 'looping'
        else:
            return 'loopFinished'
	  
	  
class Navigating2Object(smach_ros.SimpleActionState):
    """ navigating to the object location according to the given task 
    Outcomes
    --------
        succeeded : finished following the path 
        preempted : a cancel request by the client occured
        aborted : an error occured in the navigation action server        
	
    input_keys
    ----------
	navigation_task : the given task by the path generator (NavTask)
    """  
    def __init__(self,actionServerNS):
	smach_ros.SimpleActionState.__init__(self,actionServerNS,
						  FollowPathAction,
						  goal_slots=['navigation_task'],
						  result_cb=self.result_callback,						  
						  input_keys=['navigation_task']  
					    )
	#self.counter = 0
	
    #def goal_callback(self, userdata, goal):
      	#rospy.loginfo('Executing state Navigation2Object\n\n')
      	#nav = NavTask()
      	#nav = userdata.navigation_task.nav_tasks[self.counter]
      	#self.counter = self.counter + 1
        #return nav	
      
    def result_callback(self, userdata, status, result):
	if status == GoalStatus.SUCCEEDED:
          return 'succeeded'
	elif status == GoalStatus.PREEMPTED:
	  return 'preempted'
	else:
	  return 'aborted'	

#TODO: aerial manipulation code integrated [DONE]
class PickingObject(smach_ros.SimpleActionState):
    """ picking object at the reached hovering location 
    Outcomes
    --------
        succeeded : the object is picked 
        preempted : a cancel request by the client occured
	aborted : the object fell from the uav while picking it ( the action server should return the error or we could later make it a preemption request depending on the implementation)
    """
    
    def __init__(self):
	smach_ros.SimpleActionState.__init__(self,'aerialAction',
						  PickObjectAction,
						  goal_cb=self.goal_callback,
						  result_cb=self.result_callback
					    )
	
    def goal_callback(self, userdata, goal):
	rospy.loginfo('Executing picking object\n\n')
	goal = kuri_msgs.msg.PickObjectGoal()
	#TODO: modify this object to be taken as input from the previous state
	# it is an empty object for now (place it with an object example when we are going to test)
	obj = Object()
	goal.object_2_pick = obj
        return goal
      
    def result_callback(self, userdata, status, result):
	if status == GoalStatus.SUCCEEDED:
          return 'succeeded'
	elif status == GoalStatus.PREEMPTED:
	  return 'preempted'
	else:
	  return 'aborted'  
	  
class Navigating2DropZone(smach.State):
    """ navigating to the drop zone  
    Outcomes
    --------
        navigating : the uav is navigating to the dropping zone
        hovering : reached the hovering position at the dropping zone 
        
    """
    
    def __init__(self,sleep_t):
        smach.State.__init__(self, 
			     outcomes=['navigating','hovering'],
			    )
        self.counter = 0
	self.sleep_t=sleep_t
	
    def execute(self, userdata):
        rospy.loginfo('Executing state Navigating2DropZone\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            return 'navigating'
        else:
            return 'hovering'

class DroppingObject(smach.State):
    """ navigating to the drop zone  
    Outcomes
    --------
        dropping : droppig the object in the dropping zone
        dropped :  the object is dropped
	droppingFail : the object fell (maybe it will be replaced with ObjectFell State)
    """  
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['dropping','dropped','droppingFail'])
        self.counter = 0
	self.sleep_t=sleep_t
	
    def execute(self, userdata):
        rospy.loginfo('Executing state DroppingObject\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            return 'dropping'
        else:
            return 'dropped'
	return 'droppingFail'  




