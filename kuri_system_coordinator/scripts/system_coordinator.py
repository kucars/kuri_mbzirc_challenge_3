#!/usr/bin/env python
#
#   Copyright (C) 2006 - 2016 by                                          
#      Tarek Taha, KURI  <tataha@tarektaha.com>                           
#                                                                         
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
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
import smach
import smach_ros


class Starting(smach.State):
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['waitingforGPS','GPSFixed'],output_keys=['starting_out'])
        self.counter = 0
	self.sleep_t=sleep_t
    def execute(self, userdata):
        rospy.loginfo('Waiting for GPS Fix\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            return 'waitingforGPS'
        else:
	    userdata.starting_out = 22 #userdata example
            return 'GPSFixed'

class Exploring(smach.State):
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['moving2pose','hovering'],input_keys=['exploring_in'],output_keys=['exploring_out'])
        self.counter = 0
	self.sleep_t=sleep_t
    def execute(self, userdata):
        rospy.loginfo('Executing state Exploring\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            return 'moving2pose'
        else:
	    userdata.exploring_out = userdata.exploring_in+1 #userdata example
            return 'hovering'

class DetectingObjects(smach.State):
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['detectingObjects','objectsDetected'],input_keys=['detecting_objects_in'],output_keys=['detecting_objects_out'] )
        self.counter = 0
	self.sleep_t=sleep_t
    def execute(self, userdata):
        rospy.loginfo('Executing state DetectingObjects\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            return 'detectingObjects'
        else:
	    userdata.detecting_objects_out=userdata.detecting_objects_in+1 #userdata example
            return 'objectsDetected'

class AllocatingTasks(smach.State):
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['planning','succeeded'], input_keys=['allocating_tasks_in'],output_keys=['allocating_tasks_out_uav1','allocating_tasks_out_uav2'])
        self.counter = 0
	self.sleep_t=sleep_t	
    def execute(self, userdata):
        rospy.loginfo('Executing state AllocatingTasks\n\n')
        userdata.allocating_tasks_out_uav1 = userdata.allocating_tasks_in + 1 #userdata example
	userdata.allocating_tasks_out_uav2 = userdata.allocating_tasks_in + 1 #userdata example
	#print(">>>>>> %f <<<<<<<<",userdata.allocating_tasks_in)
        rospy.sleep(self.sleep_t)
        if self.counter < 4:
            self.counter += 1
            return 'planning'
        else:
            return 'succeeded'

class Navigating2Object(smach.State):
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['navigating','reached'], input_keys=['navigating_2_object_in'])
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
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['descending','picked','object_fell'])
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
	return 'object_fell'  

class Navigating2DropZone(smach.State):
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['navigating','hovering'], input_keys=['navigating_2dropzone_in'])
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
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['dropping','dropped','dropping_fault'])
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
	return 'dropping_fault'  

class WaitingForTask(smach.State):
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

def main():
    rospy.init_node('kuri_mbzirc_challenge3_state_machine')

    ### Explorer UAV
    uav_exlorer_sm = smach.StateMachine(outcomes=['done'], 
					input_keys=['explorer_gps_in'],
					output_keys=['explorer_objects_out'] 
				       )
    with uav_exlorer_sm:
        smach.StateMachine.add('Exploring', Exploring(0.5),
                               transitions={
					    'moving2pose':'Exploring',
                                            'hovering':'DetectingObjects'
					   },
			       remapping={
					  'exploring_in':'explorer_gps_in',
					  'exploring_out':'waypoints'
					 }
			       )
        smach.StateMachine.add('DetectingObjects', DetectingObjects(0.5),
                               transitions={
					    'detectingObjects':'DetectingObjects',
                                            'objectsDetected':'done'
					   },
			       remapping={
					  'detecting_objects_in' : 'waypoints',
					  'detecting_objects_out':'explorer_objects_out'
					 }
			      )


    ### Task Allocator
    task_allocator_sm = smach.StateMachine(outcomes=['tasks_ready'],
					   input_keys=['task_allocator_in'],
					   output_keys=['task_allocator_out_uav1', 'task_allocator_out_uav2']
					   )
    with task_allocator_sm:
        smach.StateMachine.add('AllocatingTasks', AllocatingTasks(0.5),
                               transitions={
					    'planning':'AllocatingTasks',
					    'succeeded':'tasks_ready'
					   },
			       remapping={
					  'allocating_tasks_in':'task_allocator_in',
					  'allocating_tasks_out_uav1':'task_allocator_out_uav1',
					  'allocating_tasks_out_uav2':'task_allocator_out_uav2'
					 }
			       )



    ### UAV Worker 1
    uav_worker1_sm = smach.StateMachine(outcomes=['finished', 'task_failed'], input_keys=['uav1_worker_tasks_in'])
    with uav_worker1_sm:
        smach.StateMachine.add('WaitingForTask', WaitingForTask(0.5),
                               transitions={
					    'waiting':'WaitingForTask',
					    'ready':'Navigating2Object'
					   },
			       remapping={
					    'waiting_for_tasks_in':'uav1_worker_tasks_in',
					    'waiting_for_tasks_out':'object'
					 }
			       )
        smach.StateMachine.add('Navigating2Object', Navigating2Object(0.5),
                               transitions={
					    'navigating':'Navigating2Object',
					    'reached':'PickingObject'},
			       remapping={
					    'navigating_2_object_in':'object'
					 }
			       )
        smach.StateMachine.add('PickingObject', PickingObject(0.5),
                               transitions={
					    'descending':'PickingObject',
					    'picked':'Navigating2DropZone',
					    'object_fell':'task_failed'
					      
					      }

			       )
        smach.StateMachine.add('Navigating2DropZone', Navigating2DropZone(0.5),
                               transitions={
					    'navigating':'Navigating2DropZone',
					    'hovering':'DroppingObject'
					   }
			       )
        smach.StateMachine.add('DroppingObject', DroppingObject(0.5),
                               transitions={
					    'dropping':'DroppingObject',
					    'dropped':'finished',
					    'dropping_fault':'task_failed'
				    
					   }
			       )
    
    ### UAV Worker 2
    uav_worker2_sm = smach.StateMachine(outcomes=['finished', 'task_failed'], input_keys=['uav2_worker_tasks_in'])
    with uav_worker2_sm:
        smach.StateMachine.add('WaitingForTask', WaitingForTask(1),
                               transitions={
					    'waiting':'WaitingForTask',
					    'ready':'Navigating2Object'
					   },
			       remapping={
					    'waiting_for_tasks_in':'uav2_worker_tasks_in',
					    'waiting_for_tasks_out':'object'
					 }
			       )
        smach.StateMachine.add('Navigating2Object', Navigating2Object(1),
                               transitions={
					    'navigating':'Navigating2Object',
					    'reached':'PickingObject'},
			       remapping={
					    'navigating_2_object_in':'object'
					 }
			       )
        smach.StateMachine.add('PickingObject', PickingObject(1),
                               transitions={
					    'descending':'PickingObject',
					    'picked':'Navigating2DropZone',
					    'object_fell':'task_failed'
					      
					      }

			       )
        smach.StateMachine.add('Navigating2DropZone', Navigating2DropZone(1),
                               transitions={
					    'navigating':'Navigating2DropZone',
					    'hovering':'DroppingObject'
					   }
			       )
        smach.StateMachine.add('DroppingObject', DroppingObject(1),
                               transitions={
					    'dropping':'DroppingObject',
					    'dropped':'finished',
					    'dropping_fault':'task_failed'
				    
					   }
			       )
    
    
    
       
    ### concurruncy
    cc = smach.Concurrence(
                    outcomes=['succeeded','failed'],
                    default_outcome='succeeded',
                    outcome_map = {
				    'succeeded':{'UAV_Worker1':'finished','UAV_Worker2':'finished'},
				    'failed':{'UAV_Worker1':'task_failed','UAV_Worker2':'task_failed'},
				    'failed':{'UAV_Worker1':'task_failed','UAV_Worker2':'finished'},
				    'failed':{'UAV_Worker1':'finished','UAV_Worker2':'task_failed'} 
				  }, 
                    input_keys=['cc_uav1_tasks_in','cc_uav2_tasks_in']
                    #input_keys=['cc_in']
                    )
	
    with cc:
	    ## the task allocator inside concurruncy have problem with userdata since the tasks are needed by the states of the uav_workers
	    #smach.Concurrence.add('Task_Allocator', task_allocator_sm,
				    #remapping={
						#'task_allocator_in':'cc_in',
						#'task_allocator_out_uav1':'UAV1_tasks',
						#'task_allocator_out_uav2':'UAV2_tasks'
					      #}			        
				  #)
            smach.Concurrence.add('UAV_Worker1', uav_worker1_sm,
				    remapping={
						'uav1_worker_tasks_in':'cc_uav1_tasks_in'
						#'uav1_worker_tasks_in':'UAV1_tasks'
					      }					       
				  
				  )
            smach.Concurrence.add('UAV_Worker2', uav_worker2_sm,
				    remapping={
						'uav2_worker_tasks_in':'cc_uav2_tasks_in'
						#'uav2_worker_tasks_in':'UAV2_tasks'
					      }					  

				  )



    ### MAIN
    main_sm = smach.StateMachine(outcomes=['Mission_Accomplished'])
    with main_sm:
        smach.StateMachine.add('Start', Starting(1),
				    transitions={'waitingforGPS':'Start',
                                        'GPSFixed':'UAV_Explorer'},
				    remapping={'starting_out':'uav_gps_loc'}
			      )		   
        smach.StateMachine.add('UAV_Explorer', uav_exlorer_sm,
				    transitions={'done':'Task_Allocator'}, #change CC to task_allocator if you want to remove taskallocator from the concurruncy
				    remapping={
						'explorer_gps_in':'uav_gps_loc',
						'explorer_objects_out':'objects_locations'
					      }
			      )
				    
	#if we don't want to put the task_allocator in the concurruncy			    
	smach.StateMachine.add('Task_Allocator', task_allocator_sm,
				    transitions={'tasks_ready':'CC'},
				    remapping={
						'task_allocator_in':'objects_locations',
						'task_allocator_out_uav1':'UAV1_tasks',
  						'task_allocator_out_uav2':'UAV2_tasks'
					      }			        
			      )
 
        smach.StateMachine.add('CC', cc,
                               transitions={'succeeded':'Mission_Accomplished',
					    'failed':'UAV_Explorer'
					   },
                               remapping={
					    'cc_in':'objects_locations',
   					    'cc_uav1_tasks_in':'UAV1_tasks',
   					    'cc_uav2_tasks_in':'UAV2_tasks'
					 }
                              )



    sis = smach_ros.IntrospectionServer('kuri_mbzirc_challenge3_state_machine_viewer', main_sm, '/kuri_mbzirc_challenge3_state_machine_root')
    sis.start()
    outcome = main_sm.execute()
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()

