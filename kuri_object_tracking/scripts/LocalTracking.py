#!/usr/bin/env python
import roslib
#roslib.load_manifest('mbzirc_challenge3_image_analysis')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from pylab import *
from datetime import datetime
import time
import os
import os
import glob
from kuri_msgs.msg import *
from os.path import expanduser

import mavros

from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler

from ObjectDetector import object_detector
from ObjectTracker import object_tracker

##Tracker
import cv
import numpy
from utils import cv2array, array2img
from tracker import skl, warpImg, maxLikelihood
from polygon import Polygon

home = expanduser("~")
nframe = 0
numberOfObject = 0
InitialX = 28
InitialY = 65

class object_tracking:

  
  def __init__(self):
    #self.objects_pub = rospy.Publisher('kuri_msgs/ObjectsMap', ObjectsMap, queue_size=5,latch=True)
    self.bridge = CvBridge()
    self.currentPoseX = 1
    self.currentPoseY = 1
    self.currentPoseZ = 1
    self.tracked_objects = []
    self.detector = object_detector()
    
    self.edgeThresholdSize = 40
    self.object_number = 1

  def draw_label(self, image, label, contour):
    im = image.copy()
    size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .8, 2)[0]
    x,y,w,h = cv2.boundingRect(contour)
    cv2.putText(image,label + "#" + ("%d" % objectIndex) + "(" + ("%.2fm" % x) + "," + ("%.2fm" % y) + ")", (x,y + 50), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 0, 0), 2)
    return image

  def updatePosition(self, topic):
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
      
  def Process(self, image2Analyse):
    global saveFolder
    global nframe
    global InitialX, InitialY
    #saveFolder = '/home/buti/images/uav3/'
    #try:
    #  cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #except CvBridgeError, e:
    #  print e
    nframe = nframe + 1
    #image2Analyse = cvImage.copy()    
    #imgfile = saveFolder + ("%05d" % nframe) + '.png'
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
    
    for ob in obstacles:
        overlap = 0
        for tracker in self.tracked_objects:
            overlap = overlap + self.overlap(ob, tracker.polygon.corners())
        if overlap == 0:
            if self.nearEdge2(ob) != True:
                new_tracker = object_tracker(ob[0], ob[1], ob[2], ob[3], self.object_number)
                self.tracked_objects.append(new_tracker)
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
    # Just send it once for the demo

       #self.objects_pub.publish(self.sentObstacles)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(image2Analyse, "bgr8"))
    #except CvBridgeError, e:
    #  print e
     
    
def main(args):
  ic = object_tracking()
  for x in range(800, 2000):
	saveFolder = '/home/buti/images/uav3/'
	imgfile = saveFolder + ("%05d" % x) + '.png'
	img = cv2.imread(imgfile)
	#cv2.imshow("Tracker", img)
	ic.Process(img)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
 
