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

from math import *
from mavros.utils import *
from geometry_msgs.msg import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from kuri_msgs.msg import *
from nav_msgs.msg import *

import smach
import smach_ros

class InitTestingMode(smach.State):
    """ Initializes state machine running mode either in component testing or full scenario  
    Outcomes
    --------
    TODO: will add more tests once the scenario is completely defined
        normalRun 		: start normal simulation scenario
        testExplorer 		: test explorer states
        tastTaskAllocator 	: test task allocator
        testUAVWorkers 		: test the uav workers concurrent

    output_keys
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
					  'uav_worker1_test_in',
					  'uav_worker2_test_in'
					 ]
                             )
	self.detectedObjects = Objects()
	self.uav1_tasks = Tasks()
	self.uav2_tasks = Tasks()	
	
    def execute(self, userdata):
	if userdata.testing_type_in is not 'normalRun':
	    #explorer start position for the (for testing waypoint generation explorer )
	    startPose = Pose()
	    startPose.position.x = 0
	    startPose.position.y = 1
	    startPose.position.z = 2
	    userdata.explorer_test_in = startPose #userdata example
	    
	    #example of the objects detected (for testing task allocator)
	    objectMap = ObjectsMap()
	    objectMap.map = OccupancyGrid()
	    self.addObjects(5) #example of 5 red objects
	    objectMap.objects = self.detectedObjects
	    userdata.task_allocator_test_in = objectMap ##userdata example
	    
	    ##example of the tasks allocated for uav1 and uav2 (for testing uav worker)
	    self.addTasksToUAVs()
	    userdata.uav_worker1_test_in = self.uav1_tasks
	    userdata.uav_worker2_test_in = self.uav2_tasks
	    
        rospy.loginfo("Running in %s mode.", userdata.testing_type_in)
        rospy.sleep(0.5)
	return userdata.testing_type_in
      
    def addObjects(self, numOfObjects):   
      i=0
      while i<numOfObjects:
	pose = PoseWithCovariance()
	pose.pose.position.x = 0+i
	pose.pose.position.y = 2+(2*i)
	pose.pose.position.z = 2	
	newObject = Object()
	newObject.pose = pose
	newObject.velocity = Twist()
	newObject.width = 1
	newObject.height = 1
	newObject.color = 'RED'
	self.detectedObjects.objects.append(newObject)
	i=i+1

    def addTasksToUAVs(self):   
      j=0
      for objectt in self.detectedObjects.objects:
	task = Task()
	task.object = objectt
	if(j%2==0):
	  task.uav_id = 2
	  self.uav2_tasks.tasks.append(task)
	else:
	  task.uav_id = 1  
	  self.uav1_tasks.tasks.append(task)
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
	  
	  
class AllocatingTasks(smach.State):
    """ allocate tasks for each UAV according to the detected objects
    Outcomes
    --------
        planning : planning the tasks for each UAV based on the objects location and the uavs locations
        tasksAllocated : the tasks are allocated for each UAV

    input_keys
    ----------
	allocating_tasks_in : detected objects locations (local/gps)
	
    output_keys
    ----------
	allocating_tasks_out_uav1 : the tasks allocated to uav worker 1 
	allocating_tasks_out_uav2 : the tasks allocated to uav worker 2 
    """  
    def __init__(self,sleep_t):
        smach.State.__init__(self, 
			     outcomes=['planning','tasksAllocated','allocationFailure'],
			     input_keys=['allocating_tasks_in'],
			     output_keys=['allocating_tasks_out_uav1','allocating_tasks_out_uav2']
			     )
        self.counter = 0
	self.sleep_t=sleep_t
	
    def execute(self, userdata):
        rospy.loginfo('Executing state AllocatingTasks\n\n')
        userdata.allocating_tasks_out_uav1 = 4 #userdata example
	userdata.allocating_tasks_out_uav2 = 3 #userdata example
	#print(">>>>>> %f <<<<<<<<",userdata.allocating_tasks_in)
        rospy.sleep(self.sleep_t)
        if self.counter < 4:
            self.counter += 1
            return 'planning'
        else:
            return 'tasksAllocated'
	return 'allocationFailure'


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
	  