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

def explorer_cc_cb(outcome_map):
   if outcome_map['EXPLORING'] == 'succeeded' and outcome_map['DETECTING_OBJECTS'] == 'succeeded' :
      return 'finished'
   elif (outcome_map['EXPLORING'] == 'aborted' and outcome_map['DETECTING_OBJECTS'] == 'succeeded') or (outcome_map['EXPLORING'] == 'aborted' and outcome_map['DETECTING_OBJECTS'] == 'preempted'):
      return 'exploring_aborted'
   elif (outcome_map['EXPLORING'] == 'succeeded' and outcome_map['DETECTING_OBJECTS'] == 'aborted') or (outcome_map['EXPLORING'] == 'preempted' and outcome_map['DETECTING_OBJECTS'] == 'aborted'):
      return 'detection_aborted'    
   elif (outcome_map['EXPLORING'] == 'preempted' and outcome_map['DETECTING_OBJECTS'] == 'succeeded'):
      return 'exploring_preempted'
   elif (outcome_map['EXPLORING'] == 'succeeded' and outcome_map['DETECTING_OBJECTS'] == 'preempted'):
      return 'detection_preempted'
   elif (outcome_map['EXPLORING'] == 'aborted' and outcome_map['DETECTING_OBJECTS'] == 'aborted'):
      return 'aborted'
   elif (outcome_map['EXPLORING'] == 'preempted' and outcome_map['DETECTING_OBJECTS'] == 'preempted'):
      return 'preempted'
    

def workers_cc_cb(outcome_map):
   if outcome_map['UAV_Worker1'] == 'workerDone' and outcome_map['UAV_Worker2'] == 'workerDone' :
      return 'workersFinished'
   elif outcome_map['UAV_Worker1'] == 'objectFellFailure' and outcome_map['UAV_Worker2'] == 'workerDone' :
      return 'uav1Failed'
   elif outcome_map['UAV_Worker1'] == 'uav1NavigationPreempted' and outcome_map['UAV_Worker2'] == 'workerDone' :
      return 'uav1Preempted' 
   elif outcome_map['UAV_Worker1'] == 'uav1NavigationFailed' and outcome_map['UAV_Worker2'] == 'workerDone' :
      return 'uav1Failed'    
   elif outcome_map['UAV_Worker1'] == 'workerDone' and outcome_map['UAV_Worker2'] == 'objectFellFailure' :
      return 'uav2Failed'
   elif outcome_map['UAV_Worker1'] == 'uav1NavigationPreempted' and outcome_map['UAV_Worker2'] == 'objectFellFailure' :
      return 'uav2Failed'    
   elif outcome_map['UAV_Worker1'] == 'uav1NavigationFailed' and outcome_map['UAV_Worker2'] == 'objectFellFailure' :
      return 'uav1Failed'
   elif outcome_map['UAV_Worker1'] == 'objectFellFailure' and outcome_map['UAV_Worker2'] == 'uav2NavigationPreempted' :
      return 'uav1Failed'    
   elif outcome_map['UAV_Worker1'] == 'objectFellFailure' and outcome_map['UAV_Worker2'] == 'uav2NavigationFailed' :
      return 'uav2Failed'      
   elif outcome_map['UAV_Worker1'] == 'workerDone' and outcome_map['UAV_Worker2'] == 'uav2NavigationPreempted' :
      return 'uav1Preempted'
   elif outcome_map['UAV_Worker1'] == 'workerDone' and outcome_map['UAV_Worker2'] == 'uav2NavigationFailed' :
      return 'uav2Failed'
   elif outcome_map['UAV_Worker1'] == 'uav1NavigationPreempted' and outcome_map['UAV_Worker2'] == 'uav2NavigationFailed' :
      return 'uav2Failed'     
   elif outcome_map['UAV_Worker1'] == 'uav1NavigationFailed' and outcome_map['UAV_Worker2'] == 'uav2NavigationFailed' :
      return 'uavsFailed'     
   elif outcome_map['UAV_Worker1'] == 'uav1NavigationPreempted' and outcome_map['UAV_Worker2'] == 'uav2NavigationPreempted' :
      return 'uavsPreempted'    
    
    
def main(testing_mode):
    rospy.init_node('kuri_mbzirc_challenge3_state_machine')


    ### MAIN
    main_sm = smach.StateMachine(outcomes=['mission_accomplished',
					   'mission_incomplete',
					   'test_succeeded', 
					   'test_failed']
				)
    with main_sm:
	"""
	********************************************************************
	* choose one of these modes:					   *
	*  1- normalRun 		: runs the normal scenario         *
        *  2- testExplorer 		: test explorer states		   *
        *  3- tastTaskAllocator 	: test task allocator	           *
        *  4- testUAVWorkers 		: test the uav workers concurrent  *
	********************************************************************
	"""
	main_sm.userdata.testing_type = testing_mode
      
	### Explorer UAV
	uav_exlorer_sm = smach.StateMachine(outcomes=['finally',
						      'detect_exp_failed',
						      'detect_exp_preempted',
						      'wp_preempted',
						      'wp_failed',
						      'path_follow_preempted',
						      'path_follow_failed',
						      'object_tracking_preempted', 
						      'object_tracking_failed'], 
					    input_keys=['explorer_gps_in'],
					    output_keys=['explorer_objects_out'] 
					   )
	### task allocator 
	task_allocator_sm = smach.StateMachine(outcomes=['tasksReady','allocationPreempted','allocationFailed'],
					      input_keys=['task_allocator_in'],
					      output_keys=['task_allocator_out'] 
					      )	
	### Worker1 UAV				    	
	uav_worker1_sm = smach.StateMachine(outcomes=['workerDone','objectFellFailure','uav1NavigationPreempted','uav1NavigationFailed'], 
					    input_keys=['uav_worker1_sm_in'],
					   )
	### Worker2 UAV			   
	uav_worker2_sm = smach.StateMachine(outcomes=['workerDone','objectFellFailure','uav2NavigationPreempted','uav2NavigationFailed'], 
					    input_keys=['uav_worker2_sm_in'],
					   )
	
	#state 1 in the main sm --> initialization 	       	
        smach.StateMachine.add('INITIALIZATION', InitTestingMode(),
				    transitions={
						  'normalRun' : 'STARTING',
						  'testExplorer' : 'TEST_UAV_EXPLORER',
						  'tastTaskAllocator' : 'TEST_TASK_ALLOCATOR',
						  'testUAVWorkers' : 'TEST_PATH_GENERATOR'
						 },
				    remapping={
						'testing_type_in' : 'testing_type',
						'explorer_test_in' : 'explorer_gps_in',
						'task_allocator_test_in':'task_allocator_in',
						'path_generator_test_in':'tasks'
						#'uav_worker1_test_in':'generating_navpaths_uav1_out',
						#'uav_worker2_test_in':'generating_navpaths_uav2_out'
					      }
			      )	
				    
	#state 2 in the main sm --> starting 	       	
	smach.StateMachine.add('STARTING', Starting(1),
				    transitions={
						  'waitingforGPS':'STARTING',
						  'GPSFixed':'UAV_EXPLORER'
						},
				    remapping={'starting_out':'uav_gps_loc'}
			      )	
				    
	#state 3 in the main sm --> explorer	       
        smach.StateMachine.add('UAV_EXPLORER', uav_exlorer_sm,
				    transitions={
						 'finally':'TASK_ALLOCATION',#mission_accomplished
						 'wp_preempted':'mission_incomplete',
						 'wp_failed':'mission_incomplete',
						 'path_follow_preempted':'mission_incomplete',
						 'path_follow_failed':'mission_incomplete',
						 'object_tracking_preempted':'mission_incomplete',
						 'object_tracking_failed':'mission_incomplete',
						 'detect_exp_failed':'mission_incomplete',
						 'detect_exp_preempted':'mission_incomplete'
						}, 
				    remapping={
						'explorer_gps_in':'uav_gps_loc',
						'explorer_objects_out':'objects_locations'
					      }
			      )
	
	#state 4 in the main sm --> task allocation
	smach.StateMachine.add('TASK_ALLOCATION', task_allocator_sm,
				    transitions={'tasksReady':'GENERATING_NAVPATHS',
						 'allocationFailed' : 'mission_incomplete',
						 'allocationPreempted' : 'mission_incomplete'
						},
				    remapping={
						'task_allocator_in':'objects_locations',
						'task_allocator_out':'uavs_tasks',
					      }			        
			      )
	
	smach.StateMachine.add('GENERATING_NAVPATHS', GeneratePaths(),
				transitions={
				  	    'succeeded':'UAV_WORKERS_CC',
					    'preempted':'mission_incomplete',
					    'aborted' : 'mission_incomplete'
					    },
				remapping={
					    'tasks':'uavs_tasks',
					    'generating_navpaths_uav1_out':'uav1_nav_path',
					    'generating_navpaths_uav2_out':'uav2_nav_path'
					  }
			      )			    
	
	
	#for now, UAVFailed corresponds to object fail failure until we add other failures types
	uav_workers_cc = smach.Concurrence(outcomes=['workersFinished','uav1Failed','uav2Failed','uav1Preempted','uav2Preempted','uavsFailed','uavsPreempted'],
					   default_outcome='workersFinished',
					   outcome_cb = workers_cc_cb,
					   input_keys=['uav_worker1_cc_in','uav_worker2_cc_in']
					  )				    
	#state 5 in the main sm--> workers concurruncy 			    
        smach.StateMachine.add('UAV_WORKERS_CC', uav_workers_cc,
				    transitions={
						 'workersFinished':'mission_accomplished',
						 'uav1Failed':'TASK_ALLOCATION',
						 'uav2Failed':'TASK_ALLOCATION',
						 'uavsFailed':'TASK_ALLOCATION',						 
						 'uav1Preempted':'mission_incomplete',
						 'uav2Preempted':'mission_incomplete',	
						 'uavsPreempted':'mission_incomplete'						 
						}, 
				    remapping={
						'uav_worker1_cc_in':'uav1_nav_path',
						'uav_worker2_cc_in':'uav2_nav_path'
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
	
	
	#************************************
	# testing states
	#TODO: add more testing states when you progress
        smach.StateMachine.add('TEST_UAV_EXPLORER', uav_exlorer_sm,
                               transitions={
					      'finally':'test_succeeded',
					      'wp_preempted':'test_failed',
					      'wp_failed':'test_failed',
					      'path_follow_preempted':'test_failed',
					      'path_follow_failed':'test_failed',
					      'object_tracking_preempted':'test_failed',
					      'object_tracking_failed':'test_failed',
					      'detect_exp_failed':'test_failed',
					      'detect_exp_preempted':'test_failed'
					   }
			      )

        smach.StateMachine.add('TEST_TASK_ALLOCATOR', task_allocator_sm,
                               transitions={
					    'tasksReady' : 'test_succeeded',
                                            'allocationPreempted' : 'test_failed',
                                            'allocationFailed' : 'test_failed'
                                           }
			      )

        smach.StateMachine.add('TEST_PATH_GENERATOR', GeneratePaths(),
                               transitions={
				  	    'succeeded':'TEST_UAV_WORKERS', #change it to TEST_UAV_WORKERS
					    'preempted':'test_failed',
					    'aborted' : 'test_failed'
                                           },
				remapping={
					    'generating_navpaths_uav1_out':'uav1_nav_path',
					    'generating_navpaths_uav2_out':'uav2_nav_path'
					  }			       
			      )

        smach.StateMachine.add('TEST_UAV_WORKERS', uav_workers_cc,
                               transitions={
					      'workersFinished':'test_succeeded',
					      'uav1Failed':'test_failed',
					      'uav2Failed':'test_failed',
					      'uavsFailed':'test_failed',						 
					      'uav1Preempted':'test_failed',
					      'uav2Preempted':'test_failed',	
					      'uavsPreempted':'test_failed'
					   },
			      remapping={
					 'uav_worker1_cc_in':'uav1_nav_path',
					 'uav_worker2_cc_in':'uav2_nav_path'
					}			       
			      )
	# END of define testing states		       
	#************************************
		    
				    
				    
				    
        #**********************************
	# Define the explorer state machine
	with uav_exlorer_sm:	
	    
	  smach.StateMachine.add('GENERATING_WAYPOINTS',  GenerateWaypoints(), 
                               transitions={
                                            'succeeded':'EXPLORE_DETECT_CC',
                                            'aborted':'wp_failed',
                                            'preempted':'wp_preempted'
					   },
			       remapping={
					  'generate_waypoints_in':'explorer_gps_in',
					  'generate_waypoints_out':'waypoints'
					 }
			        )
			       
	  explore_detect_cc = smach.Concurrence(outcomes=['finished','exploring_aborted','exploring_preempted','detection_aborted','detection_preempted','aborted','preempted'],
						default_outcome='aborted',
						outcome_cb = explorer_cc_cb,
						input_keys=['explore_detect_cc_in'],
						output_keys=['explore_detect_cc_out']
						)		       
	  smach.StateMachine.add('EXPLORE_DETECT_CC', explore_detect_cc,
			           transitions={
						'finished':'finally',
						'aborted':'detect_exp_failed',
						'preempted':'detect_exp_preempted',
						'exploring_aborted':'path_follow_failed',
						'exploring_preempted':'path_follow_preempted',
						'detection_aborted':'object_tracking_failed',
						'detection_preempted':'object_tracking_preempted'
					       },
				   remapping={
					      'explore_detect_cc_in':'waypoints',
					      'explore_detect_cc_out':'explorer_objects_out'
					     }
				)


	  #*************************************
	  # Define the explor_detect concurruncy
	  with explore_detect_cc:
	   smach.Concurrence.add('EXPLORING', Exploring('/uav_1/navigation_action_server1'),
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
	  smach.StateMachine.add('ALLOCATING_TASKS', AllocatingTasks(),
				    transitions={
						 'succeeded':'tasksReady',
						 'aborted' : 'allocationFailed',
						 'preempted':'allocationPreempted'
						}, 
				    remapping={
						'objects_map':'task_allocator_in',
						'allocating_tasks_out':'task_allocator_out'
					      }
			        )				    
	
	# End task_allocator state machine  
	#************************************
	
	
	
	
	
	#*************************************
	# Define the uav_worker1 state machine
	with uav_worker1_sm:
	  #smach.StateMachine.add('LOOPING_NAVTASKS', NavTasksLoop(),
				#transitions={
				  	    #'loopFinished':'workerDone',
					    #'looping':'NAVIGATING_2_OBJECT',
					    #},
				#remapping={
					    #'looping_in':'uav_worker1_sm_in',
					    #'looping_out':'nav_task'
					  #}
				#)	  
	  smach.StateMachine.add('NAVIGATING_2_OBJECT', Navigating2Object('/uav_2/navigation_action_server2'),
				transitions={
				  	    'succeeded':'PICKING_OBJECT',
					    'preempted':'uav1NavigationPreempted',
					    'aborted' : 'uav1NavigationFailed'
					    },
				remapping={
					    'navigation_task':'uav_worker1_sm_in'
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
					    'dropped':'workerDone',
					    'droppingFail':'OBJECT_FELL'
					    }
				)
				
	  smach.StateMachine.add('OBJECT_FELL', ObjectFell(0.5),
				transitions={
					    'canSee':'PICKING_OBJECT',
					    'cannotSee':'objectFellFailure'
					    }
			      )			
				
	# End uav_worker1 state machine  
	#*************************************




	#*************************************
	# Define the uav_worker2 state machine
	with uav_worker2_sm:
	  #smach.StateMachine.add('LOOPING_NAVTASKS', NavTasksLoop(),
				#transitions={
				  	    #'loopFinished':'workerDone',
					    #'looping':'NAVIGATING_2_OBJECT',
					    #},
				#remapping={
					    #'looping_in':'uav_worker2_sm_in',
					    #'looping_out':'nav_task'
					  #}
				#)	  
	  smach.StateMachine.add('NAVIGATING_2_OBJECT', Navigating2Object('/uav_3/navigation_action_server3'),
				transitions={
				  	    'succeeded':'PICKING_OBJECT',
					    'preempted':'uav2NavigationPreempted',
					    'aborted' : 'uav2NavigationFailed'
					    },
				remapping={
					    'navigation_task':'uav_worker2_sm_in'
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
					    'dropped':'workerDone',
					    'droppingFail':'OBJECT_FELL'
					    }
				)
	  smach.StateMachine.add('OBJECT_FELL', ObjectFell(0.5),
				transitions={
					    'canSee':'PICKING_OBJECT',
					    'cannotSee':'objectFellFailure'
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
  if len(sys.argv) < 2:
    print(" Not enough arguments, please choose from the below testing modes: \n - normalRun \n - testExplorer \n - tastTaskAllocator \n - testUAVWorkers")
    print('Defaulting to normal run mode.')
    main('normalRun')
  else:  
    main(sys.argv[1])

