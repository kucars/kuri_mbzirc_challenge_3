#! /usr/bin/env python
#Copyright (c) 2016, Buti Al Delail
#All rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are met:
#
#* Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#* Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#* Neither the name of kuri_mbzirc_challenge_3 nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,

import rospy
import thread
import threading
import time
import mavros
import actionlib
from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from kuri_msgs.msg import *

class dropzone_landing:

    def __init__(self):
        self.done = False
        self.done_evt = threading.Event()
        self.isExploring = False
        self.progress = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.currentPoseX = 0 
        self.currentPoseY = 0
        self.currentPoseZ = 0
        self.navigating = False
        
        mavros.set_namespace('/uav_1/mavros')
        # publisher for mavros/setpoint_position/local
        self.pub = SP.get_pub_position_local(queue_size=10)
        # subscriber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    SP.PoseStamped, self.reached)

        self.objects_map = ObjectsMap()

        self.client = actionlib.SimpleActionClient('TrackingAction', TrackingAction)
        #client = self.client
        #client = self.actionServer.client

        print "Waiting for tracking server"
        self.client.wait_for_server()
        self.goal = TrackingGoal()
        self.goal.uav_id = 1
        self.client.send_goal(self.goal)
        print "Waiting for result"
        self.client.wait_for_result()
        print "Result:"
        self.objects =self.client.get_result().tracked_objects.objects
        print self.objects

        try:
            thread.start_new_thread(self.navigate, ())
        except:
            fault("Error: Unable to start thread")

    def navigate(self):
        rate = rospy.Rate(40)   # 10hz

        msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
        )

        while not rospy.is_shutdown():
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z

            # For demo purposes we will lock yaw/heading to north.
            yaw_degrees = 0  # North
            yaw = radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation = SP.Quaternion(*quaternion)
                        
            self.pub.publish(msg)
            rate.sleep()

    def setPose(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z
        self.navigating = True
        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()
        time.sleep(delay)
        
    def takeoff(self, z, delay=0, wait=True):
        diff = z - self.currentPoseZ
        while not abs(diff)<0.2:
            diff = z - self.currentPoseZ
            if diff>0:
                self.setPose(self.currentPoseX,self.currentPoseY,self.currentPoseZ + 1, 0, False)
            else:
                self.setPose(self.currentPoseX,self.currentPoseY,self.currentPoseZ - 0.1, 0, False)
    
    def land(self, delay=0, wait=True):
        altitude = self.currentPoseZ
        while altitude > 0:
            altitude = self.currentPoseZ
            self.setPose(self.currentPoseX,self.currentPoseY,self.currentPoseZ - 0.5 ,2)
    
    def reached(self, topic):
        def is_near(msg, x, y, d):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            return abs(x - y) < d
        self.currentPoseX = topic.pose.position.x 
        self.currentPoseY = topic.pose.position.y
        self.currentPoseZ = topic.pose.position.z
        
        if is_near('X', topic.pose.position.x, self.x, 0.2) and \
           is_near('Y', topic.pose.position.y, self.y, 0.2) and \
           is_near('Z', topic.pose.position.z, self.z, 0.5):
            if  self.navigating:
                self.done = True
                self.navigating = False
                self.done_evt.set()


    def explore(self):
        print 'explore started '
        rate = rospy.Rate(30)
        self.newGoal = True
        if self.isExploring == False:
            #Change this later when we have a better exploration
            #self.isExploring = True
            while self.done == False:
                    time.sleep(1)
                    rospy.loginfo("Climb")
                    self.progress += 0.1
                    self.takeoff(5)
                    self.progress += 0.1
                    rospy.loginfo("Moving to Red_Object")
                    self.reached_object = False
                    red_object_id = -1
                    xspeed = 1
                    while self.reached_object == False:
                        self.client.send_goal(self.goal)
                        self.client.wait_for_result()
                        self.objects = self.client.get_result().tracked_objects.objects
                        islost = True
                        for obj in self.objects:
                            if red_object_id == -1 and (obj.color == 'RED' or obj.color == 'BLUE' or obj.color == 'GREEN'): #pick any nearby object
                                red_object_id = obj.object_id
                            if obj.object_id == red_object_id:
                                islost = False
                                print 'Moving to Drop zone', self.currentPoseX-obj.pose2.pose.position.x, self.currentPoseY-obj.pose2.pose.position.y, obj.pose.pose.position.x, obj.pose.pose.position.y
                                if fabs(obj.pose2.pose.position.x) < 0.01 and fabs(obj.pose2.pose.position.y) > 0.01:
                                    print 'Moving Y'
                                    self.setPose(self.x, self.currentPoseY+obj.pose2.pose.position.y*xspeed, self.z, 0 , False)
                                elif fabs(obj.pose2.pose.position.y) < 0.01 and fabs(obj.pose2.pose.position.x) > 0.01:
                                    print 'Moving X'
                                    self.setPose(self.currentPoseX-obj.pose2.pose.position.x*xspeed, self.y, self.z, 0 , False)
                                else:
                                    print 'Moving XY'
                                    self.setPose(self.currentPoseX-obj.pose2.pose.position.x*xspeed, self.currentPoseY+obj.pose2.pose.position.y*xspeed, self.z, 0 , True)
                                if fabs(obj.pose2.pose.position.x) < 0.3 and fabs(obj.pose2.pose.position.y) < 0.3 and self.z > 0.0:
                                    print 'Moving Z'
                                    land = 0.2
                                    if self.z <= 3:
                                        xspeed = 1
                                    if self.z <= 1.5:
                                        xspeed = 0.5
                                    if self.z < 0.5:
                                        land = 0.05
                                    self.setPose(self.x, self.y, self.z - land * xspeed, 1, False)
                                    if self.z <= 0.4:
                                        self.reached_object = True
                        if islost == True:
                            red_object_id = -1
                        if red_object_id == -1:
                            rospy.loginfo("No object in sight, exploring")
                            #self.setPose(self.x, self.y - 5, self.z, 1, True)
                        rate.sleep()   
                    time.sleep(10)                    
                    rospy.loginfo("Picked Object, climb")
                    self.takeoff(1)
                    self.takeoff(2)
                    self.takeoff(3)
                    self.takeoff(4)
                    self.takeoff(5)
                    #self.setPose(self.x, self.y, self.z)
                    time.sleep(10)
                    rospy.loginfo("Moving to DropZone")
                    self.setPose(1, -21, 5) ##Go near dropzone
                    self.progress += 0.1
                    self.reached_dropzone = False
                    xspeed = 3
                    while self.reached_dropzone == False:
                        self.client.send_goal(self.goal)
                        self.client.wait_for_result()
                        self.objects = self.client.get_result().tracked_objects.objects
            
                        for obj in self.objects:
                            if obj.color == 'DROP_ZONE':
                                print 'Moving to Drop zone', self.currentPoseX-obj.pose2.pose.position.x, self.currentPoseY-obj.pose2.pose.position.y, obj.pose.pose.position.x, obj.pose.pose.position.y
                                if fabs(obj.pose2.pose.position.x) < 0.1 and fabs(obj.pose2.pose.position.y) > 0.1:
                                    print 'Moving Y'
                                    self.setPose(self.x, self.currentPoseY+obj.pose2.pose.position.y*xspeed, self.z, 0 , False)
                                elif fabs(obj.pose2.pose.position.y) < 0.1 and fabs(obj.pose2.pose.position.x) > 0.1:
                                    print 'Moving X'
                                    self.setPose(self.currentPoseX-obj.pose2.pose.position.x*xspeed, self.y, self.z, 0 , False)
                                else:
                                    print 'Moving XY'
                                    self.setPose(self.currentPoseX-obj.pose2.pose.position.x*xspeed, self.currentPoseY+obj.pose2.pose.position.y*xspeed, self.z, 0 , True)
                                if fabs(obj.pose2.pose.position.x) < 0.3 and fabs(obj.pose2.pose.position.y) < 0.3 and self.z > 1:
                                    print 'Moving Z'
                                    land = 0.5
                                    if self.z <= 3:
                                        land = 0.2
                                        xspeed = 0.5
                                    self.setPose(self.x, self.y, self.z - land, 1, False)
                                    if self.z < 1.5:
                                        self.reached_dropzone = True
                        rate.sleep()
                    self.progress += 0.1
                    rospy.loginfo("Landed Object, climb")
                    self.takeoff(7)
                    rospy.loginfo("Bye!")

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('dropzone_landing', anonymous=True)
    d = dropzone_landing()
    d.explore()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
