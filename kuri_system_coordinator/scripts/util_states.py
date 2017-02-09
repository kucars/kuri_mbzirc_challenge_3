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
from nav_msgs.msg import *
from kuri_msgs.msg import GenerateExplorationWaypointsAction, Task, NavTask,NavTasks, FollowPathAction,TrackingAction,GeneratePathsAction
from smach_ros import SimpleActionState
from actionlib_msgs.msg import GoalStatus


class InitTestingMode(smach.State):
    """ Initializes state machine running mode either in component testing or full scenario  
    Outcomes
    --------
    TODO: will add more tests once the scenario is completely defined
        normalRun 		: start normal simulation scenario
        testExplorer 		: test explorer states
        tastTaskAllocator 	: test task allocator
        testUAVWorkers 		: test the uav workers concurrent

    input_keys
    ----------
	testing_type_in : the chosen testing mode type
	
    output_keys
    ----------
	explorer_test_in : the starting position (recived from the starting state)
	task_allocator_test_in : task allocator goal (ObjectsMap) 
	uav_worker1_test_in : uav 1 tasks (objects locations)
	uav_worker2_test_in : uav 2 tasks (objects locations)
    """

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['normalRun',
                                       'testExplorer',
                                       'tastTaskAllocator',
                                       'testUAVWorkers'],
                             input_keys=['testing_type_in'],
                             output_keys=[
					  'explorer_test_in',
					  'task_allocator_test_in',
					  'tasks',
					  'uav_worker1_test_in',
					  'uav_worker2_test_in'
					 ]
                             )
	self.detectedObjects = Objects()
	#self.uav1_tasks = Tasks()
	#self.uav2_tasks = Tasks()
	self.uavs_tasks = Tasks()	
        self.objectMap = ObjectsMap()

	
    def execute(self, userdata):
	if userdata.testing_type_in is not 'normalRun':
	    #explorer start position for the (for testing waypoint generation explorer )
	    startPose = Pose()
	    startPose.position.x = 0
	    startPose.position.y = 1
	    startPose.position.z = 2
	    userdata.explorer_test_in = startPose #userdata example
	    
	    #example of the objects detected (for testing task allocator)
	    self.objectMap.map = OccupancyGrid()
	    self.addObjects(2) #example of 2 red objects
	    userdata.task_allocator_test_in = self.objectMap ##userdata example
	    
	    ##example of the tasks allocated for uav1 and uav2 (for testing uav worker)
	    ##IMPORTANT : it should be one task per UAV 
	    self.addTasksToUAVs()
	    userdata.tasks = self.uavs_tasks
	    #userdata.uav_worker1_test_in = self.uav1_tasks
	    #userdata.uav_worker2_test_in = self.uav2_tasks
	    
        rospy.loginfo("Running in %s mode.", userdata.testing_type_in)
        rospy.sleep(0.5)
	return userdata.testing_type_in
      
    def addObjects(self, numOfObjects):   
      i=0
      j=-1
      r=0
      while i<numOfObjects:
	pose = PoseWithCovariance()
	pose.pose.position.x = i*j
	pose.pose.position.y = 2+(r*j)
	pose.pose.position.z = 10
	print("object %i: x:%f, y:%f, z:%f" % (i,pose.pose.position.x,pose.pose.position.y,pose.pose.position.z))
	newObject = Object()
	newObject.pose = pose
	newObject.velocity = Twist()
	newObject.width = 100
	newObject.height = 100
	newObject.color = 'red' ## TODO: it is used in the object tracker and the taskallocator codes
	self.detectedObjects.objects.append(newObject)
	self.objectMap.objects.append(newObject)
	i=i+1
	r=r+3
	j=j*-1

    def addTasksToUAVs(self):   
      j=0
      for objectt in self.detectedObjects.objects:
	task = Task()
	task.object = objectt
	if(j%2==0):
	  task.uav_id = 2
	  task.uav_name = 'UAV2'
	  self.uavs_tasks.tasks.append(task)	  
	  #self.uav2_tasks.tasks.append(task)
	else:
	  task.uav_id = 3 
	  task.uav_name = 'UAV3'
	  self.uavs_tasks.tasks.append(task)	  
	  #self.uav1_tasks.tasks.append(task)
	j=j+1

	
class Starting(smach.State):
    """ wait until GPS is fixed
    Outcomes
    --------
        waitingforGPS : waiting the gps to get fixed
        GPSFixed : the gps is fixed, it is done
	
    output_keys
    ----------
	starting_out : the current global position of the uav (gps)
    """  
    def __init__(self,sleep_t):
        smach.State.__init__(self, 
			     outcomes=['waitingforGPS','GPSFixed'],
			     output_keys=['starting_out'])
        self.counter = 0
	self.sleep_t=sleep_t
	
    def execute(self, userdata):
        rospy.loginfo('Waiting for GPS Fix\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            return 'waitingforGPS'
        else:
	    startPose = Pose()
	    startPose.position.x = 0
	    startPose.position.y = 1
	    startPose.position.z = 22
	    userdata.starting_out = startPose #userdata example
            return 'GPSFixed'
	  
	  
class AllocatingTasks(smach_ros.SimpleActionState):
    """ allocate tasks for each UAV according to the detected objects
    Outcomes
    --------
        succeeded : tasks planning is done for each UAV based on the objects location and maps
        preempted : a cancel request by the client occured
        aborted : an error occured in the task allocation action server

    input_keys
    ----------
	allocating_tasks_in : detected objects locations (local/gps)
	
    output_keys
    ----------
	allocating_tasks_out : the tasks allocated to uavs
    """  
    def __init__(self):
	smach_ros.SimpleActionState.__init__(self,'actionAllocationServ',
						  AllocateTasksAction,
						  #goal_cb=self.goal_callback,
						  result_cb=self.result_callback,
						  goal_slots=['objects_map'],
						  input_keys=['objects_map'], 
						  output_keys=['allocating_tasks_out']
					    )      

      
    def result_callback(self, userdata, status, result):
      	rospy.loginfo('Executing state AllocatingTasks\n\n')
	if status == GoalStatus.SUCCEEDED:
	  userdata.allocating_tasks_out = result.tasks
          return 'succeeded'
	elif status == GoalStatus.PREEMPTED:
	  return 'preempted'
	else:
	  return 'aborted'


class ObjectFell(smach.State):
    """ the object fell either at picking object, going to drop zone, or dropping object  
    Outcomes
    --------
        canSee : can see the object after it fell
        cannotSee :  cannot see the object after it fell

    """  
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['canSee','cannotSee'])
        self.counter = 0
	self.sleep_t=sleep_t
	
    def execute(self, userdata):
        rospy.loginfo('Executing state ObjectFell\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            return 'canSee'
        else:
            return 'cannotSee'


class GeneratePaths(smach_ros.SimpleActionState):
    """ navigating to the object location according to the given task 
    Outcomes
    --------
        navigating : going to the hovering position of the object
        reached : reached the hovering place of the object
	
    input_keys
    ----------
	navigating_2_object_in : the given task by the task allocator ( I'll assume it gives object position )
    """  
    def __init__(self):
	smach_ros.SimpleActionState.__init__(self,'path_planner_action_server',
						  GeneratePathsAction,
						  goal_slots=['tasks'],
						  result_cb=self.result_callback,						  
						  input_keys=['tasks'],
						  output_keys=['generating_navpaths_uav1_out','generating_navpaths_uav2_out']
					    )
    def result_callback(self, userdata, status, result):
      	rospy.loginfo('Executing state Navigating2Object\n\n')

	if status == GoalStatus.SUCCEEDED:
	  #navUAV1Tasks = NavTasks()
	  #navUAV2Tasks = NavTasks()  
	  print("number of generated paths: %f",len(result.nav_tasks.nav_tasks))
	  for navPath in result.nav_tasks.nav_tasks:
	    if navPath.task.uav_id == 2:
	      print("***UAV2***")
	      #navUAV1Tasks.nav_tasks.append(navPath)
	      userdata.generating_navpaths_uav1_out = navPath
	    elif navPath.task.uav_id == 3:
	      print("***UAV3***")
	      userdata.generating_navpaths_uav2_out = navPath
	      #navUAV2Tasks.nav_tasks.append(navPath)
	      
	  #userdata.generating_navpaths_uav1_out = navUAV1Tasks
	  #userdata.generating_navpaths_uav2_out = navUAV2Tasks	
          return 'succeeded'
	elif status == GoalStatus.PREEMPTED:
	  return 'preempted'
	else:
	  return 'aborted'	  