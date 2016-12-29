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
    def __init__(self):
        smach.State.__init__(self, outcomes=['waitingforGPS','GPSFixed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Waiting for GPS Fix')
        rospy.sleep(1.0)
        if self.counter < 3:
            self.counter += 1
            return 'waitingforGPS'
        else:
            return 'GPSFixed'

class Exploring(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving2pose','hovering'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('CON 1: Executing state Exploring')
        rospy.sleep(1.0)
        if self.counter < 3:
            self.counter += 1
            return 'moving2pose'
        else:
            return 'hovering'

class DetectingObjects(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detectingObjects','objectsDetected'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('CON 1: Executing state DetectingObjects')
        rospy.sleep(1.0)
        if self.counter < 3:
            self.counter += 1
            return 'detectingObjects'
        else:
            return 'objectsDetected'

class AllocatingTasks(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['planning','succeeded'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state AllocatingTasks')
        rospy.sleep(1.0)
        if self.counter < 3:
            self.counter += 1
            return 'planning'
        else:
            return 'succeeded'

class Navigating2Object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigating','reached'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Navigating2Object')
        rospy.sleep(1.0)
        if self.counter < 3:
            self.counter += 1
            return 'navigating'
        else:
            return 'reached'

class PickingObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['descending','picked'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PickingObject')
        rospy.sleep(1.0)
        if self.counter < 3:
            self.counter += 1
            return 'descending'
        else:
            return 'picked'

class Navigating2DropZone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['navigating','hovering'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Navigating2DropZone')
        rospy.sleep(1.0)
        if self.counter < 3:
            self.counter += 1
            return 'navigating'
        else:
            return 'hovering'

class DroppingObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dropping','dropped'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state DroppingObject')
        rospy.sleep(1.0)
        if self.counter < 3:
            self.counter += 1
            return 'dropping'
        else:
            return 'dropped'

class WaitingForTask(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['waiting','ready'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state DroppingObject')
        rospy.sleep(1.0)
        if self.counter < 3:
            self.counter += 1
            return 'waiting'
        else:
            return 'ready'

def main():
    rospy.init_node('kuri_mbzirc_challenge3_state_machine')


    explorationUAV_SM = smach.StateMachine(outcomes=['done'])
    with explorationUAV_SM:
        smach.StateMachine.add('Exploring', Exploring(),
                               transitions={'moving2pose':'Exploring',
                                            'hovering':'DetectingObjects'})
        smach.StateMachine.add('DetectingObjects', DetectingObjects(),
                                    transitions={'detectingObjects':'DetectingObjects',
                                                 'objectsDetected':'done'})

    task_allocator_sm = smach.StateMachine(outcomes=['tasks_ready'])
    with task_allocator_sm:
        smach.StateMachine.add('AllocatingTasks', AllocatingTasks(),
                               transitions={'planning':'AllocatingTasks',
                                'succeeded':'tasks_ready'})

    uav_worker = smach.StateMachine(outcomes=['finished'])
    with uav_worker:
        smach.StateMachine.add('WaitingForTask', WaitingForTask(),
                               transitions={'waiting':'WaitingForTask',
                                'ready':'Navigating2Object'})
        smach.StateMachine.add('Navigating2Object', Navigating2Object(),
                               transitions={'navigating':'Navigating2Object',
                                'reached':'PickingObject'})
        smach.StateMachine.add('PickingObject', PickingObject(),
                               transitions={'descending':'PickingObject',
                                'picked':'Navigating2DropZone'})
        smach.StateMachine.add('Navigating2DropZone', Navigating2DropZone(),
                               transitions={'navigating':'Navigating2DropZone',
                                'hovering':'DroppingObject'})
        smach.StateMachine.add('DroppingObject', DroppingObject(),
                               transitions={'dropping':'DroppingObject',
                               'dropped':'finished'})

        cc = smach.Concurrence(
                    outcomes=['JustDoIt'],
                    default_outcome='JustDoIt',
                    outcome_map = {'JustDoIt':{'MainState':'finished','ExplorationUAV_SM':'done'}})

        with cc:
            smach.Concurrence.add('MainState', uav_worker)
            smach.Concurrence.add('ExplorationUAV_SM', explorationUAV_SM)


    main_sm = smach.StateMachine(outcomes=['WeWin'])
    with main_sm:
        smach.StateMachine.add('Start', Starting(),
                           transitions={'waitingforGPS':'Start',
                                        'GPSFixed':'CC'})
        smach.StateMachine.add('ExplorationUAV_SM', explorationUAV_SM,
                                   transitions={'done':'WeWin'})
        smach.StateMachine.add('MainState', uav_worker,
                           transitions={'finished':'WeWin'})
        smach.StateMachine.add('CC', cc,
                               transitions={'JustDoIt':'WeWin'})

    sis = smach_ros.IntrospectionServer('kuri_mbzirc_challenge3_state_machine_viewer', main_sm, '/kuri_mbzirc_challenge3_state_machine_root')
    sis.start()
    outcome = main_sm.execute()
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()

