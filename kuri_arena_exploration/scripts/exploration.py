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
from explore_action_server import ExploreServer

class Exploration:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        # TODO(simon): Clean this up.
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
        # publisher for mavros/setpoint_position/local
        self.pub = SP.get_pub_position_local(queue_size=10)
        # subscriber for mavros/local_position/local
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'),
                                    SP.PoseStamped, self.reached)

        #self.actionServer = ExploreServer()
        self.objects_map = ObjectsMap()

        self.client = actionlib.SimpleActionClient('MappingAction', MappingAction)
        client = self.client
        #client = self.actionServer.client
        
        print "Waiting for mapping server"
        client.wait_for_server()        
        goal = TrackingGoal()
        goal.uav_id = 3
        client.send_goal(goal)
        print "Waiting for result"
        client.wait_for_result() 
        print "Result:"
        self.objects_map = client.get_result().objects_map
        print self.objects_map        
        
        self.sub = rospy.Subscriber("MappingAction/feedback",ObjectsMap, self.callback)


        
    def callback(self, objects):
        print 'Recieving Mapped Objects Progress ', objects.feedback.total_mapped_objects
        #self.objects_map = objects.feedback.total_mapped_objects#.objects_map
        
        if self.actionServer.hasGoal:
            self.actionServer.update(self.objects_map)
        else:
            self.actionServer.objects_map = self.objects_map
            self.actionServer._feedback.area_percent_complete = len(self.objects_map.objects)

        try:
            thread.start_new_thread(self.navigate, ())
        except:
            fault("Error: Unable to start thread")


    def navigate(self):
        rate = rospy.Rate(10)   # 10hz

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
                self.setPose(self.currentPoseX,self.currentPoseY,self.currentPoseZ + 1 ,2)
            else:
                self.setPose(self.currentPoseX,self.currentPoseY,self.currentPoseZ - 1 ,2)
    
    def land(self, delay=0, wait=True):
        altitude = self.currentPoseZ
        while altitude > 0:
            altitude = self.currentPoseZ
            self.setPose(self.currentPoseX,self.currentPoseY,self.currentPoseZ - 1 ,2)
    
    def reached(self, topic):
        def is_near(msg, x, y):
            rospy.logdebug("Position %s: local: %d, target: %d, abs diff: %d",
                           msg, x, y, abs(x - y))
            return abs(x - y) < 0.2
        self.currentPoseX = topic.pose.position.x 
        self.currentPoseY = topic.pose.position.y
        self.currentPoseZ = topic.pose.position.z
        
        if is_near('X', topic.pose.position.x, self.x) and \
           is_near('Y', topic.pose.position.y, self.y) and \
           is_near('Z', topic.pose.position.z, self.z):
            self.done = True
            self.done_evt.set()


    def explore():
        print 'explore started '
        if self.isExploring == False:
            self.isExploring = True
            while self.done == False: 
                rate = rospy.Rate(10)
            
                setpoint = SetpointPosition()
                
                time.sleep(1)
                
                rospy.loginfo("Climb")
                setpoint.progress += 0.1
                setpoint.takeoff(15)
                setpoint.progress += 0.1
                rospy.loginfo("Moving to Pose 1")
                setpoint.progress += 0.1
                setpoint.setPose(0,0,15,5)
                setpoint.progress += 0.1    
                #rospy.loginfo("Landing")
                #setpoint.land()
            
                rospy.loginfo("Bye!")
            self.isExploring = False

