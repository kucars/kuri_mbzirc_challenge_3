#!/usr/bin/env python
#
#   Copyright (C) 2006 - 2016 by                                          
#      Tarek Taha, KURI  <tataha@tarektaha.com>                           
#      Randa Almadhoun, KURI <randa.almadhoun@gmail.com> 
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
import actionlib
import smach
import smach_ros

from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from uav_explorer_states import *
from util_states import *
from uav_worker_states import *
from kuri_msgs.msg import GenerateExplorationWaypointsAction, Task
from smach_ros import SimpleActionState

def main():
    rospy.init_node('kuri_mbzirc_challenge3_state_machine')


    ### MAIN
    main_sm = smach.StateMachine(outcomes=['mission_accomplished','mission_incomplete'])
    with main_sm:
      
	### Explorer UAV
	uav_exlorer_sm = smach.StateMachine(outcomes=['finally','smth_wrong'], 
					    input_keys=['explorer_gps_in'],
					    output_keys=['explorer_objects_out'] 
					   )
	### task allocator 
	task_allocator_sm = smach.StateMachine(outcomes=['tasksReady','fail'],
					    input_keys=['task_allocator_in'],
					    output_keys=['task_allocator_out_uav1','task_allocator_out_uav2'] 
					   )	
	### Worker1 UAV				    	
	uav_worker1_sm = smach.StateMachine(outcomes=['taskDone','ObjectFellFailure'], 
					    input_keys=['uav_worker1_sm_in'],
					   )
	### Worker2 UAV			   
	uav_worker2_sm = smach.StateMachine(outcomes=['taskDone','ObjectFellFailure'], 
					    input_keys=['uav_worker2_sm_in'],
					   )
	
	
	
	#state 1 in the main sm --> starting 	       	
	smach.StateMachine.add('STARTING', Starting(1),
				    transitions={
						  'waitingforGPS':'STARTING',
						  'GPSFixed':'UAV_EXPLORER'
						},
				    remapping={'starting_out':'uav_gps_loc'}
			      )	
				    
	#state 2 in the main sm --> explorer	       
        smach.StateMachine.add('UAV_EXPLORER', uav_exlorer_sm,
				    transitions={
						 'finally':'TASK_ALLOCATION',#mission_accomplished
						 'smth_wrong':'mission_incomplete'
						}, 
				    remapping={
						'explorer_gps_in':'uav_gps_loc',
						'explorer_objects_out':'objects_locations'
					      }
			      )
	
	#state 3 in the main sm --> task allocation
	smach.StateMachine.add('TASK_ALLOCATION', task_allocator_sm,
				    transitions={'tasksReady':'UAV_WORKERS_CC',
						 'fail' : 'mission_incomplete'
						},
				    remapping={
						'task_allocator_in':'objects_locations',
						'task_allocator_out_uav1':'uav1_tasks',
  						'task_allocator_out_uav2':'uav2_tasks'
					      }			        
			      )	
        #smach.StateMachine.add('ALLOCATING_TASKS', AllocatingTasks(0.5),
				    #transitions={
						 #'planning':'ALLOCATING_TASKS',
						 #'tasksAllocated':'UAV_WORKERS_CC'#mission_accomplished
						#}, 
				    #remapping={
						#'allocating_tasks_in':'objects_locations',
						#'allocating_tasks_out_uav1':'uav1_tasks',
						#'allocating_tasks_out_uav2':'uav2_tasks'
					      #}
			      #)
	
	#for now, UAVFailed corresponds to object fail failure until we add other failures types
	uav_workers_cc = smach.Concurrence(outcomes=['workersFinished','UAVFailed'],
					   default_outcome='workersFinished',
					   outcome_map = {
							    'workersFinished':{'UAV_Worker1':'taskDone','UAV_Worker2':'taskDone'},
							    'UAVFailed':{'UAV_Worker1':'ObjectFellFailure','UAV_Worker2':'taskDone'},
							    'UAVFailed':{'UAV_Worker1':'taskDone','UAV_Worker2':'ObjectFellFailure'},
							    'UAVFailed':{'UAV_Worker1':'ObjectFellFailure','UAV_Worker2':'ObjectFellFailure'}
							  }, 
					   input_keys=['uav_worker1_cc_in','uav_worker2_cc_in']
					  )				    
	#state 4 in the main sm--> workers concurruncy 			    
        smach.StateMachine.add('UAV_WORKERS_CC', uav_workers_cc,
				    transitions={
						 'workersFinished':'mission_accomplished',
						 'UAVFailed':'TASK_ALLOCATION'
						}, 
				    remapping={
						'uav_worker1_cc_in':'uav1_tasks',
						'uav_worker2_cc_in':'uav2_tasks'
					      }
			      )
	
	#*************************************
	# Define the uav_workers concurruncy	
	with uav_workers_cc:
            smach.Concurrence.add('UAV_Worker1', uav_worker1_sm,
				    remapping={
						'uav_worker1_sm_in':'uav_worker1_cc_in'
					      }					       
				  
				 )
            smach.Concurrence.add('UAV_Worker2', uav_worker2_sm,
				    remapping={
						'uav_worker2_sm_in':'uav_worker2_cc_in'
					      }					  

				 )
	# End uav_workers concurruncy  
	#************************************			    
				    
			    
				    
				    
				    
        #**********************************
	# Define the explorer state machine
	with uav_exlorer_sm:
	  #smach.StateMachine.add('STARTING', Starting(1),
				    #transitions={
						  #'waitingforGPS':'STARTING',
						  #'GPSFixed':'GENERATING_WAYPOINTS'
						#},
				    #remapping={'starting_out':'uav_gps_loc'}
			        #)	
	    
	  smach.StateMachine.add('GENERATING_WAYPOINTS',  GenerateWaypoints(), 
                               transitions={
                                            'succeeded':'EXPLORE_DETECT_CC',
                                            'aborted':'smth_wrong',
                                            'preempted':'smth_wrong'
					   },
			       remapping={
					  'generate_waypoints_in':'explorer_gps_in',
					  'generate_waypoints_out':'waypoints'
					 }
			        )
			       
	  explore_detect_cc = smach.Concurrence(outcomes=['finished','aborted'],
						default_outcome='finished',
						outcome_map = {
								'finished':{'EXPLORING':'succeeded','DETECTING_OBJECTS':'succeeded'},
								#'explore_detect':{'EXPLORING':'moving2Pose','DETECTING_OBJECTS':'detectingObjects'},
								'aborted':{'EXPLORING':'aborted','DETECTING_OBJECTS':'succeeded'},
								'aborted':{'EXPLORING':'succeeded','DETECTING_OBJECTS':'aborted'},
								'aborted':{'EXPLORING':'aborted','DETECTING_OBJECTS':'aborted'},
								'aborted':{'EXPLORING':'preempted','DETECTING_OBJECTS':'aborted'},
								'aborted':{'EXPLORING':'preempted','DETECTING_OBJECTS':'succeeded'},
								'aborted':{'EXPLORING':'aborted','DETECTING_OBJECTS':'preempted'},
								'aborted':{'EXPLORING':'succeeded','DETECTING_OBJECTS':'preempted'},
								'aborted':{'EXPLORING':'preempted','DETECTING_OBJECTS':'preempted'}
							      }, 
						input_keys=['explore_detect_cc_in'],
						output_keys=['explore_detect_cc_out']
						)		       
	  smach.StateMachine.add('EXPLORE_DETECT_CC', explore_detect_cc,
			           transitions={
						'finished':'finally',
						#'explore_detect':'EXPLORE_DETECT_CC',
						'aborted':'smth_wrong'
					       },
				   remapping={
					      'explore_detect_cc_in':'waypoints',
					      'explore_detect_cc_out':'explorer_objects_out'
					     }
				)


	  #*************************************
	  # Define the explor_detect concurruncy
	  with explore_detect_cc:
	   smach.Concurrence.add('EXPLORING', Exploring(),
			       remapping={
					  'navigation_task':'explore_detect_cc_in',
					 }
			        )
	   smach.Concurrence.add('DETECTING_OBJECTS', DetectingObjects(),
			       remapping={
					  'detecting_objects_out':'explore_detect_cc_out'
					 }
			        )
	  # End explore_detect conccuruncy  
	  #************************************
	# End uav_explorer state machine  
	#************************************
	
	
	
	
	
	#*************************************
	# Define the task_allocator state machine	
	with task_allocator_sm:
	  smach.StateMachine.add('ALLOCATING_TASKS', AllocatingTasks(0.5),
				    transitions={
						 'planning':'ALLOCATING_TASKS',
						 'tasksAllocated':'tasksReady',
						 'allocationFailure' : 'fail'
						}, 
				    remapping={
						'allocating_tasks_in':'task_allocator_in',
						'allocating_tasks_out_uav1':'task_allocator_out_uav1',
						'allocating_tasks_out_uav2':'task_allocator_out_uav2'
					      }
			      )
	
	# End task_allocator state machine  
	#************************************
	
	
	
	
	
	#*************************************
	# Define the uav_worker1 state machine
	with uav_worker1_sm:
	  smach.StateMachine.add('NAVIGATING_2_OBJECT', Navigating2Object(0.5),
				transitions={
					    'navigating':'NAVIGATING_2_OBJECT',
					    'reached':'PICKING_OBJECT'
					    },
				remapping={
					    'navigating_2_object_in':'uav1_worker_sm_in'
					  }
				)
	  smach.StateMachine.add('PICKING_OBJECT', PickingObject(0.5),
				transitions={
					    'descending':'PICKING_OBJECT',
					    'picked':'NAVIGATING_2_DROPZONE',
					    'pickFail':'OBJECT_FELL' 
					    }
				)
	  smach.StateMachine.add('NAVIGATING_2_DROPZONE', Navigating2DropZone(0.5),
				transitions={
					    'navigating':'NAVIGATING_2_DROPZONE',
					    'hovering':'DROPPING_OBJECT'
					   }
				)
	  smach.StateMachine.add('DROPPING_OBJECT', DroppingObject(0.5),
				transitions={
					    'dropping':'DROPPING_OBJECT',
					    'dropped':'taskDone',
					    'droppingFail':'OBJECT_FELL'
					    }
				)
				
	  smach.StateMachine.add('OBJECT_FELL', ObjectFell(0.5),
				transitions={
					    'canSee':'PICKING_OBJECT',
					    'cannotSee':'ObjectFellFailure'
					    }
			      )			
				
	# End uav_worker1 state machine  
	#*************************************




	#*************************************
	# Define the uav_worker2 state machine
	with uav_worker2_sm:
	  smach.StateMachine.add('NAVIGATING_2_OBJECT', Navigating2Object(0.5),
				transitions={
					    'navigating':'NAVIGATING_2_OBJECT',
					    'reached':'PICKING_OBJECT'
					    },
				remapping={
					    'navigating_2_object_in':'uav2_worker_sm_in'
					  }
				)
	  smach.StateMachine.add('PICKING_OBJECT', PickingObject(0.5),
				transitions={
					    'descending':'PICKING_OBJECT',
					    'picked':'NAVIGATING_2_DROPZONE',
					    'pickFail':'OBJECT_FELL' 
					    }
				)
	  smach.StateMachine.add('NAVIGATING_2_DROPZONE', Navigating2DropZone(0.5),
				transitions={
					    'navigating':'NAVIGATING_2_DROPZONE',
					    'hovering':'DROPPING_OBJECT'
					    }
				)
	  smach.StateMachine.add('DROPPING_OBJECT', DroppingObject(0.5),
				transitions={
					    'dropping':'DROPPING_OBJECT',
					    'dropped':'taskDone',
					    'droppingFail':'OBJECT_FELL'
					    }
				)
	  smach.StateMachine.add('OBJECT_FELL', ObjectFell(0.5),
				transitions={
					    'canSee':'PICKING_OBJECT',
					    'cannotSee':'ObjectFellFailure'
					    }
			      )				
	# End uav_worker2 state machine  
	#************************************





    # Create the introspection server
    sis = smach_ros.IntrospectionServer('kuri_mbzirc_challenge3_state_machine_viewer', main_sm, '/kuri_mbzirc_challenge3_state_machine_root')
    sis.start()
    
    # Execute the main state machine
    outcome = main_sm.execute()
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()

