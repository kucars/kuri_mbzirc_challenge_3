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
from os.path import expanduser
home = expanduser("~")

class image_converter:

  
  def __init__(self):
    global violationImagesFolder
    rospy.init_node('mbzirc_challenge3_image_analysis', anonymous=True)
    self.image_pub = rospy.Publisher("/uav_2/downward_cam/image_output",Image, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/uav_2/downward_cam/camera/image",Image,self.callback)
        
  def callback(self,data):
    
    try:
      cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
      
    image2Analyse = cvImage.copy()
    (rows,cols,channels) = image2Analyse.shape
    element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    
    gray = cv2.cvtColor(image2Analyse,cv2.COLOR_BGR2GRAY) # convert to gray scale
    #eroded = cv2.erode(gray,element) # erode the image
    #edg = cv2.Canny(eroded,50,50) # detect edges canny
    filtered = cv2.adaptiveThreshold(gray,  255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 21, 1)
    #image, contours, hierarchy = cv2.findContours(filtered, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) # find contours
    #cv2.drawContours(cnt, contours, -1, (0,255,0), 3)
    #cv2.imshow('Contours',cnt)
    
    im2 = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)

    # define the list of boundaries for blue color
    lower = np.array([110, 150, 150])
    upper = np.array([130, 255, 255])    
    # find the colors within the specified boundaries and apply the mask
    mask   = cv2.inRange(im2, lower, upper)
    output = cv2.bitwise_and(im2, im2, mask = mask)
    cv2.imshow('Filtered',output)        
    
    image2Analyse = cvImage.copy()
    cv2.putText(image2Analyse,"Just a Test", (50, rows / 2), cv2.FONT_HERSHEY_SIMPLEX, 2, (0,0,255),2)
    cv2.imshow('Testing',image2Analyse)
    
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(image2Analyse, "bgr8"))
    except CvBridgeError, e:
      print e
     
    
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
 
