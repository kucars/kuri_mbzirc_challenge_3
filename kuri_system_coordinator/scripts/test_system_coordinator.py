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
   

def main_cc_term_cb(outcome_map):
  if(outcome_map['STATUS_CHECKING']=='theEnd'):
    return True
  else :
    return False

def main_cc_cb(outcome_map):
  if(outcome_map['STATUS_CHECKING']=='theEnd'):
    return 'comp_done'
  else:
    return 'working'  
    
def main(testing_mode):
    rospy.init_node('kuri_mbzirc_challenge3_state_machine')
    start= float(str(time.time()))#rospy.get_time()   # stamp should update
    rospy.loginfo(">>>>>>>>>>>>>>>>>>>>>>>>>>>start time : %f" , (start))

    ### MAIN
    main_sm = smach.StateMachine(outcomes=['mission_accomplished',
					   'mission_incomplete',
					   'test_succeeded', 
					   'test_failed']
				)
    with main_sm:
	"""
	#TODO testing modes are deleted (only choose normal run) --> later we will return them back
	********************************************************************
	* choose one of these modes:					   *
	*  1- normalRun 		: runs the normal scenario         *
        *  2- testExplorer 		: test explorer states		   *
        *  3- tastTaskAllocator 	: test task allocator	           *
        *  4- testUAVWorkers 		: test the uav workers concurrent  *
	********************************************************************
	"""
	main_sm.userdata.testing_type = testing_mode
      
	
	### exploration
	exploration_sm = smach.StateMachine(outcomes=['exp_failed','exp_preempted'],
					      input_keys=['exploration_sm_in']
					   )	
	### detection and tracking 
	detection_sm = smach.StateMachine(outcomes=['tracker_failed','tracker_preempted']  )

	### mapping 	
	mapping_sm = smach.StateMachine(outcomes=['mapping_failed','mapping_preempted']  )	

	### task allocator 
	task_allocator_sm = smach.StateMachine(outcomes=['tasksReady','allocationPreempted','allocationFailed'],
					      output_keys=['task_allocator_out1','task_allocator_out2'] 
					      )	
	

	### Worker1 UAV				    	
	uav_worker1_sm = smach.StateMachine(outcomes=['workerDone','objectFellFailure','uav1NavigationPreempted','uav1NavigationFailed'], 
					    input_keys=['uav_worker1_sm_in'],
					   )
	### Worker2 UAV			   
	uav_worker2_sm = smach.StateMachine(outcomes=['workerDone','objectFellFailure','uav2NavigationPreempted','uav2NavigationFailed'], 
					    input_keys=['uav_worker2_sm_in'],
					   )
	
	
	
	
	
	###state 1 in the main sm --> initialization 	
	#TODO return back the testing modes ( it was removed since everything is running parallel)
        smach.StateMachine.add('INITIALIZATION', InitTestingMode(),
				    transitions={
						  'normalRun' : 'STARTING'
						  #'testExplorer' : 'TEST_GENERATING_WAYPOINTS',
						  #'tastTaskAllocator' : 'TEST_TASK_ALLOCATOR',
						  #'testUAVWorkers' : 'TEST_UAV_WORKERS'
						 },
				    remapping={
						'testing_type_in' : 'testing_type',
						'waypoint_generator_test_in' : 'generate_waypoints_in',
						'task_allocator_test_in':'task_allocator_in',
						'path_generator_test_in':'tasks'
						#'uav_worker1_test_in':'generating_navpaths_uav1_out',
						#'uav_worker2_test_in':'generating_navpaths_uav2_out'
					      }
			      )	
				    
	###state 2 in the main sm --> starting 	       	
	smach.StateMachine.add('STARTING', Starting(1),
				    transitions={
						  'waitingforGPS':'STARTING',
						  'GPSFixed':'GENERATING_WAYPOINTS'
						},
				    remapping={'starting_out':'uav_gps_loc'}
			      )	

	###state 3 in the main sm --> generating waypoints 	       		
	#smach.StateMachine.add('GENERATING_WAYPOINTS',  GenerateWaypoints(), 
				    #transitions={
						  #'succeeded':'All_CC', #UAV_EXPLORER
						  #'aborted':'mission_incomplete',
						  #'preempted':'mission_incomplete'
						#},
				    #remapping={
						#'generate_waypoints_in':'uav_gps_loc',
						#'generate_waypoints_out':'waypoints'
					      #}
			      #)	

	smach.StateMachine.add('GENERATING_WAYPOINTS',  ReadWaypoints(), 
				    transitions={
						  'succeeded':'All_CC', #UAV_EXPLORER
						  'aborted':'mission_incomplete'
						},
				    remapping={
						'generate_waypoints_in':'uav_gps_loc',
						'generate_waypoints_out':'waypoints'
					      }
			      )					    


	###state 4 in the main sm --> concurruncy	       						    
	#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	explorer_allocator_workers_cc = smach.Concurrence(outcomes=['comp_done','working'],
					   default_outcome='working',
					   child_termination_cb = main_cc_term_cb,
					   outcome_cb = main_cc_cb,					   
					   input_keys=['eaw_cc_in']
					  )
	with explorer_allocator_workers_cc:
	  
           smach.Concurrence.add('STATUS_CHECKING', StatusChecking(rospy.get_time(),start) )	
    
	   smach.Concurrence.add('EXPLORE', exploration_sm,
			       remapping={
					  'exploration_sm_in':'eaw_cc_in',
					 }
			        )   			        
           smach.Concurrence.add('DETECT', detection_sm
					       
				)
	   
	   smach.Concurrence.add('MAP', mapping_sm
					       
				)
	   
	   #if (taskAllocFlag[0] == 'True'):
	   smach.Concurrence.add('TASKALLOCATION', task_allocator_sm	       
				)				    
	
	
	#THE BIG CONCURRUNCY THAT YOU WILL EVER SEE IN YOUR LIFE >_<
        smach.StateMachine.add('All_CC', explorer_allocator_workers_cc,
				    transitions={
						 'comp_done':'mission_accomplished',
						 'working':'All_CC',						 
						}, 			       
			       	    remapping={
						'eaw_cc_in':'waypoints'
					      }
			       )	
        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>	  
	  
	  
	  
			    
	#TODO TO BE checked and replaced	
	# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>remove me when you check concurrence (TO BE checked and replaced)		
	#for now, UAVFailed corresponds to object fail failure until we add other failures types
	#uav_workers_cc = smach.Concurrence(outcomes=['workersFinished','uav1Failed','uav2Failed','uav1Preempted','uav2Preempted','uavsFailed','uavsPreempted'],
					   #default_outcome='workersFinished',
					   #outcome_cb = workers_cc_cb,
					   #input_keys=['uav_worker1_cc_in','uav_worker2_cc_in']
					  #)				    
	##state 5 in the main sm--> workers concurruncy 			    
        #smach.StateMachine.add('UAV_WORKERS_CC', uav_workers_cc,
				    #transitions={
						 #'workersFinished':'mission_accomplished',
						 #'uav1Failed':'TASK_ALLOCATION',
						 #'uav2Failed':'TASK_ALLOCATION',
						 #'uavsFailed':'TASK_ALLOCATION',						 
						 #'uav1Preempted':'mission_incomplete',
						 #'uav2Preempted':'mission_incomplete',	
						 #'uavsPreempted':'mission_incomplete'						 
						#}, 
				    #remapping={
						#'uav_worker1_cc_in':'uav1_task',
						#'uav_worker2_cc_in':'uav2_task'
					      #}
			      #)

	##*************************************
	## Define the uav_workers concurruncy	
	#with uav_workers_cc:
            #smach.Concurrence.add('UAV_Worker1', uav_worker1_sm,
				    #remapping={
						#'uav_worker1_sm_in':'uav_worker1_cc_in'
					      #}					       
				  
				 #)
            #smach.Concurrence.add('UAV_Worker2', uav_worker2_sm,
				    #remapping={
						#'uav_worker2_sm_in':'uav_worker2_cc_in'
					      #}					  
				 #)
	## End uav_workers concurruncy  
	##************************************			    
	
	
		    

        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	# Define the exploreration machine
	#TODO: assign namespaces for each physical drone
	#TODO: confirm that after gps homming the local position is set to 0,0,0 (in real experiments)
	with exploration_sm:
	  smach.StateMachine.add('EXPLORING', Exploring('/uav_1/navigation_action_server1'),
                              transitions={
                                            'succeeded':'EXPLORING',
                                            'aborted':'exp_failed',
                                            'preempted':'exp_preempted'
					   },			 
			       remapping={
					  'navigation_task':'exploration_sm_in',
					 }
				) 	  


        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	# Define the detection machine
	#TODO: add latch after the publish
	#TODO: objects detector should return a list of objects and return an objects list even if it was one
	with detection_sm:
	  smach.StateMachine.add('TRACKING', DetectingObjects(),
                              transitions={
                                            'succeeded':'TRACKING',
                                            'aborted':'tracker_failed',
                                            'preempted':'tracker_preempted'
					   }			
				) 
			      
        #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	# Define the mapping machine
	with mapping_sm:
	  smach.StateMachine.add('MAPPING', Mapping(),
                              transitions={
                                            'succeeded':'MAPPING',
                                            'aborted':'mapping_failed',
                                            'preempted':'mapping_preempted'
					   }			
				)
				    
				    
	#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	# Define the task_allocator state machine
	#TODO: it should genarate new tasks only if the workers are not performing a task
	#TODO: it should be deployed in one system (centralized system, ground station) --> task allocator, path generator, and statemachine
	with task_allocator_sm:
	    smach.StateMachine.add('ALLOCATING_TASKS', AllocatingTasks(),
				      transitions={
						  'succeeded':'GENERATING_NAVPATHS',
						  'aborted' : 'allocationFailed',
						  'preempted':'allocationPreempted'
						  }, 
				      remapping={
						  'allocating_tasks_out':'uavs_tasks'
						}
				  )
	    smach.StateMachine.add('GENERATING_NAVPATHS', GeneratePaths(),
				transitions={
				  	    'succeeded':'tasksReady',
					    'preempted':'allocationFailed',
					    'aborted' : 'allocationPreempted'
					    },
				remapping={
					    'tasks':'uavs_tasks',
					    'generating_navpaths_uav1_out':'task_allocator_out1',
					    'generating_navpaths_uav2_out':'task_allocator_out2'
					  }
				  )		
	# End task_allocator state machine  
	#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	
	
	
	
	
	#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	# Define the uav_worker1 state machine
	#TODO: landing : develop object picking controller (probably use butti object tracking)
	#TODO: service call to check if the object is attached or not 
	#TODO: dropping : developing object dropping controller (butti drop box tracker)
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
	  #TODO link the picking object to aerial manipulation action server [DONE]
	  smach.StateMachine.add('PICKING_OBJECT', PickingObject(),
				transitions={
					    'succeeded':'NAVIGATING_2_DROPZONE',
					    'preempted':'OBJECT_FELL',
					    'aborted':'OBJECT_FELL' 
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
	#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>




	#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	# Define the uav_worker2 state machine
	#TODO: landing : develop object picking controller (probably use butti object tracking)
	#TODO: service call to check if the object is attached or not 
	#TODO: dropping : developing object dropping controller (butti drop box tracker)	
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
	  #TODO link the picking object to aerial manipulation action server [DONE]
	  smach.StateMachine.add('PICKING_OBJECT', PickingObject(),
				transitions={
					    'succeeded':'NAVIGATING_2_DROPZONE',
					    'preempted':'OBJECT_FELL',
					    'aborted':'OBJECT_FELL' 
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
	#>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>





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

