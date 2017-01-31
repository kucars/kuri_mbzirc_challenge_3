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
import smach
import smach_ros


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

class Navigating2Object(smach.State):
    """ navigating to the object location according to the given task 
    Outcomes
    --------
        navigating : going to the hovering position of the object
        reached : reached the hovering place of the object
	
    input_keys
    ----------
	navigating_2_object_in : the given task by the task allocator ( I'll assume it gives object position )
    """  
    def __init__(self,sleep_t):
        smach.State.__init__(self, 
			     outcomes=['navigating','reached'],
			     input_keys=['navigating_2_object_in'])
        self.counter = 0
	self.sleep_t=sleep_t
	
    def execute(self, userdata):
        rospy.loginfo('Executing state Navigating2Object\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            return 'navigating'
        else:
            return 'reached'

class PickingObject(smach.State):
    """ picking object at the reached hovering location 
    Outcomes
    --------
        descending : uav is descending to pick the object
        picked : the object is picked 
	object_fell : the object fell from the uav while picking it
    """
    
    def __init__(self,sleep_t):
        smach.State.__init__(self, 
			     outcomes=['descending','picked','pickFail']
			    )
        self.counter = 0
	self.sleep_t=sleep_t
	
    def execute(self, userdata):
        rospy.loginfo('Executing state PickingObject\n\n')
        rospy.sleep(self.sleep_t)
        
        if self.counter < 3:
            self.counter += 1
            return 'descending'
	else:
	    return 'picked'
	return 'pickFail'  
	  
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




