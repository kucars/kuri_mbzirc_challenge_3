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
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import roslib
#roslib.load_manifest('mbzirc_challenge3_image_analysis')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
import time
import os
import os
import glob
from kuri_msgs.msg import *
from geometry_msgs.msg import *
from os.path import expanduser

import mavros

from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler

home = expanduser("~")
nframe = 0
numberOfObject = 0
InitialX = 28
InitialY = 65

from object_detector import object_detector
from singleobject_tracker import object_tracker
from tracking_action_server import TrackingServer

class object_tracking:

  
  def __init__(self, actionServer):
    #cv2.namedWindow("Tracker", cv2.CV_WINDOW_AUTOSIZE)
    self.image_pub = rospy.Publisher("/uav_3/downward_cam/image_output",Image, queue_size=10)

    self.obstacles = Objects()
    self.map = ObjectsMap()
        
    self.detector = object_detector()
    
    self.edgeThresholdSize = 40
    self.object_number = 1
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/uav_3/downward_cam/camera/image",Image,self.callback)
    mavros.set_namespace('/uav_3/mavros')
    self.currentPoseX = -1
    self.currentPoseY = -1
    self.currentPoseZ = -1
    self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.updatePosition)
    self.objectsSent  = False
    self.tracked_objects = []
    self.actionServer = actionServer

  def draw_label(self, image, label, contour):
    im = image.copy()
    size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .8, 2)[0]
    x,y,w,h = cv2.boundingRect(contour)
    cv2.putText(image,label + "#" + ("%d" % objectIndex) + "(" + ("%.2fm" % x) + "," + ("%.2fm" % y) + ")", (x,y + 50), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 0, 0), 2)
    return image

  def updatePosition(self, topic):
    self.pose = PoseWithCovariance()
    self.pose.pose = topic.pose
    self.currentPoseX = topic.pose.position.x 
    self.currentPoseY = topic.pose.position.y
    self.currentPoseZ = topic.pose.position.z
      
  def overlap(self, a, b):  # returns None if rectangles don't intersect
    dx = min(a[0] + a[2], b[2][0]) - max(a[0], b[0][0])
    dy = min(a[1] + a[3], b[1][1]) - max(a[1], b[0][1])
    if (dx>=0) and (dy>=0):
        return 1#dx*dy
    return 0
    
  def nearEdge(self, a):
      if a[0] < self.edgeThresholdSize or a[1] < self.edgeThresholdSize or a[0] > (self.width - self.edgeThresholdSize) or a[1] > (self.height - self.edgeThresholdSize):
          return True
      return False
      
  def nearEdge2(self, a):
      if a[0] < self.edgeThresholdSize*2 or a[1] < self.edgeThresholdSize*2 or a[0] > (self.width - self.edgeThresholdSize*2) or a[1] > (self.height - self.edgeThresholdSize*2):
          return True
      return False
      
  def addObject(self, pose, velocity, width, height, color):
      newObject = Object()
      newObject.pose = pose
      newObject.velocity = Twist()
      #velocity
      newObject.width = width
      newObject.height = height
      newObject.color = color
      self.obstacles.objects.append(newObject)
      if self.actionServer.hasGoal:
          self.actionServer.update(self.obstacles, self.object_number + 1)
      else:
          self.actionServer.objects = self.obstacles
          print 'No Goal'
      #self.actionServer.objects = self.obstacles.objects
      #self.actionServer.total_objects = self.object_number + 1
      #self.actionServer._feedback.total_objects_tracked = self.object_number + 1
      #self.actionServer.server.publish_feedback(self.actionServer._feedback)
      #self.actionServer.update(self.obstacles, self.object_number + 1)
      
  def callback(self,data):
    global saveFolder
    global nframe
    global InitialX, InitialY
    #saveFolder = '/home/buti/images/uav3/'
    try:
      cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
    
    if self.currentPoseZ < 10:
        print 'UAV Hight is Low..'
        return
    
    nframe = nframe + 1
    print 'Processing'
    image2Analyse = cvImage.copy()    
    img = image2Analyse
    self.height, self.width = img.shape[:2]
    
    for tracker in self.tracked_objects:
        poly = tracker.track_object(img)
        result = poly.corners()
        if self.nearEdge([result[0][0], result[0][1]]) == False:
            cv2.rectangle(img, (result[0][0], result[0][1]), (result[2][0], result[2][1]), (255,0,0), 2)
            cv2.putText(img,"#" + ("%d" % tracker.id), (result[0][0],result[0][1] + 50), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 0, 0), 2)
        else:
            self.tracked_objects.remove(tracker)
    
    
    obstacles = self.detector.detect_obstacles(img)
    #colors = self.detector.detect_colors(img, obstacles)
    
    for ob in obstacles:
        overlap = 0
        for tracker in self.tracked_objects:
            overlap = overlap + self.overlap(ob, tracker.polygon.corners())
        if overlap == 0:
            if self.nearEdge2(ob) != True:
                new_tracker = object_tracker(ob[0], ob[1], ob[2], ob[3], self.object_number)
                self.tracked_objects.append(new_tracker)
                self.addObject(self.pose, 0, ob[2], ob[3], 'RED')
                self.object_number = self.object_number + 1
    
    
    label = "POS" +  "(" + ("%.2fm" % self.currentPoseX) + "," + ("%.2fm" % self.currentPoseY) + "," + ("%.2fm" % self.currentPoseZ) + ")"
    if InitialX == 0 and InitialY == 0:
	InitialX = self.currentPoseX
	InitialY = self.currentPoseY
    cv2.putText(img,label, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #cv2.imwrite(imgfile, img) 
    small = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow("Tracker", small)
    cv2.waitKey(10);

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(image2Analyse, "bgr8"))
    except CvBridgeError, e:
      print e
     
 
