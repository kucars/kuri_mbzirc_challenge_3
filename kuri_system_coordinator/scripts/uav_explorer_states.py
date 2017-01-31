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



class GenerateWaypoints(smach.State):
    """generates a set of waypoints for covering the entrire arena 
    Outcomes
    --------
        generatingWaypoints : generate the set of waypoints
        waypointsGenerated : the generation of the waypoints is done
   
    input_keys
    ----------
	generate_waypoints_in : the starting location (gps or local)
	
    output_keys
    ----------
	generate_waypoints_out : a set of waypoints the UAV should pass to entirely map the arena
    """
    
    def __init__(self,sleep_t):
        smach.State.__init__(self, 
			     outcomes=['generatingWaypoints','waypointsGenerated'],
			     input_keys=['generate_waypoints_in'],
			     output_keys=['generate_waypoints_out'])
        self.counter = 0
	self.sleep_t=sleep_t
	
    def execute(self, userdata):
        rospy.loginfo('Executing state generating waypoints\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            userdata.generate_waypoints_out = 10 #userdata example
            return 'generatingWaypoints'
	else:
	    return 'waypointsGenerated'
	  
	  
class Exploring(smach.State):
    """Explore the arena 
    Outcomes
    --------
        move2Pose : moving to the waypoint
        hovering : hovering at one of the waypoint 
    
    input_keys
    ----------
	exploring_in : the waypoint location (gps or local)
    """
    
    def __init__(self,sleep_t):
        smach.State.__init__(self, 
			     outcomes=['moving2Pose','hovering','exploreFailed'],
			     input_keys=['exploring_in']
			     )
        self.counter = 0
	self.sleep_t=sleep_t
	
    def execute(self, userdata):
        rospy.loginfo('Executing state Exploring\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            return 'moving2Pose'
        else:
            return 'hovering'

	return 'exploreFailed'

class DetectingObjects(smach.State):
    """detecting the objects at the waypoints of the exploration  
    Outcomes
    --------
        detectingObjects : execute the objects detection and tracking at the hovering place
        objectsDetected : objects detection is done for all the waypoints
    
    input_keys
    ----------
	detecting_objects_in : uav location
	
    output_keys
    ----------
	detecting_objects_out : detected objects locations
    """  
    def __init__(self,sleep_t):
        smach.State.__init__(self, outcomes=['detectingObjects','objectsDetected','detectFailed'],
				   output_keys=['detecting_objects_out'] )
        self.counter = 0
	self.sleep_t=sleep_t
	
    def execute(self, userdata):
        rospy.loginfo('Executing state DetectingObjects\n\n')
        rospy.sleep(self.sleep_t)
        if self.counter < 3:
            self.counter += 1
            userdata.detecting_objects_out=3
            return 'detectingObjects'
        else:
	    #userdata.detecting_objects_out=userdata.detecting_objects_in+1 #userdata example
	    userdata.detecting_objects_out=3 #userdata example
            return 'objectsDetected'
	return 'detectFailed'
