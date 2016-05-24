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

home = expanduser("~")
nframe = 0
numberOfObject = 0
InitialX = 28
InitialY = 65

class image_converter:

  
  def __init__(self):
    #cv2.namedWindow("Tracker", cv2.CV_WINDOW_AUTOSIZE)
    rospy.init_node('mbzirc_challenge3_cv_test', anonymous=True)
    self.image_pub = rospy.Publisher("/uav_3/downward_cam/image_output",Image, queue_size=10)

    self.objects_pub = rospy.Publisher('kuri_msgs/Objects', Objects, queue_size=5,latch=True)
    self.currentObject = Object()
    self.currentObject.width = 10 
    self.currentObject.height = 10
    self.currentObject.pose.pose.position.x = InitialX
    self.currentObject.pose.pose.position.y = InitialY;
    #self.currentObject.pose = PoseWithCovariance(pose=pose)
    self.currentObject.color = 'RED'

    self.obstacles = Objects()
    #self.obstacles.objects.append(self.currentObject)
    
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/uav_3/downward_cam/camera/image",Image,self.callback)
    mavros.set_namespace('/uav_3/mavros')
    self.currentPoseX = -1
    self.currentPoseY = -1
    self.currentPoseZ = -1
    self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.updatePosition)

  def angle_cos(self, p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

  def get_holes(self, image, thresh):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    im_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]
    im_bw_inv = cv2.bitwise_not(im_bw)

    contour, _ = cv2.findContours(im_bw_inv, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contour:
        cv2.drawContours(im_bw_inv, [cnt], 0, 255, -1)

    nt = cv2.bitwise_not(im_bw)
    im_bw_inv = cv2.bitwise_or(im_bw_inv, nt)
    return im_bw_inv


  def remove_background(self, image, thresh, scale_factor=1.0, kernel_range=range(1, 15), border=1):
    border = border or kernel_range[-1]

    holes = self.get_holes(image, thresh)
    small = cv2.resize(holes, None, fx=scale_factor, fy=scale_factor)
    bordered = cv2.copyMakeBorder(small, border, border, border, border, cv2.BORDER_CONSTANT)

    for i in kernel_range:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*i+1, 2*i+1))
        bordered = cv2.morphologyEx(bordered, cv2.MORPH_CLOSE, kernel)

    unbordered = bordered[border: -border, border: -border]
    mask = cv2.resize(unbordered, (image.shape[1], image.shape[0]))
    fg = cv2.bitwise_and(image, image, mask=mask)
    return fg

  def draw_label(self, image, label, contour, font=cv2.FONT_HERSHEY_SIMPLEX,
               font_scale=.8, thickness=2):
    im = image.copy()
    size = cv2.getTextSize(label, font, font_scale, thickness)[0]
    x,y,w,h = cv2.boundingRect(contour)
    meter_pixel = 0.015 / 2#(w+h)/0.6; 
    M = cv2.moments(contour)
    x = int(M['m10']/M['m00'])
    y = int(M['m01']/M['m00'])
    #print meter_pixel
    height, width = im.shape[:2]
    obj_x = ((width/2.0) - x) * meter_pixel
    obj_x = obj_x + self.currentPoseX
    obj_y = ((height/2.0) - y) * meter_pixel
    obj_y = obj_y + self.currentPoseY
    #cv2.putText(image,label + "(" + ("%.2fm" % obj_x) + "," + ("%.2fm" % (obj_y)) + ")", (x,y), font, font_scale, (0, 0, 255), thickness)
    x_world = self.currentPoseX + (y - (width/2.0)) * self.currentPoseZ / 800;
    y_world = self.currentPoseY + (x - (height/2.0)) * self.currentPoseZ / 800;
    if abs(InitialY-self.currentPoseY) < 10 or InitialY-self.currentPoseY < 0: ##ignores start area
	return image
    #x_world = (x_world + obj_x) / 2
    #y_world = (y_world + obj_y) / 2
    objectNum = 0
    objectIndex = 0
    minX = 1000
    minY = 1000
    minOb = Object()
    bExists = False
    for ob in self.obstacles.objects:
         objectNum = objectNum + 1
	 destX = abs(ob.pose.pose.position.x - x_world)
	 destY = abs(ob.pose.pose.position.y - y_world)
         if destX < minX and destY < minY and destX < 2 and destY < 2 and ob.color == label:
		minOb = ob
                bExists = True
		objectIndex = objectNum
                self.obstacles.objects[objectIndex-1].pose.pose.position.x = x_world
		self.obstacles.objects[objectIndex-1].pose.pose.position.y = y_world
    if bExists == False:
    	newObject = Object()
    	newObject.width = w
    	newObject.height = h
    	newObject.pose.pose.position.x = x_world
    	newObject.pose.pose.position.y = y_world
    	newObject.color = label
	self.obstacles.objects.append(newObject)
        objectIndex = objectNum
    cv2.putText(image,label + "#" + ("%d" % objectIndex) + "(" + ("%.2fm" % x_world) + "," + ("%.2fm" % y_world) + ")", (x,y + 50), font, font_scale, (255, 0, 0), thickness)
    return image

  def detect_obstacles(self, img):
    squares = []
    circles = []
    if InitialY and (InitialY-self.currentPoseY) < 10: ##ignores start area
	return img
    nobg = self.remove_background(img, 155)
    gray = cv2.cvtColor(nobg,cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
	cnt_len = cv2.arcLength(cnt, True)
        cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
	if len(cnt) >= 4 and len(cnt) <= 6 and cv2.contourArea(cnt) > 50 and cv2.isContourConvex(cnt):
		#_cnt = cnt.reshape(-1, 2)
                #max_cos = np.max([self.angle_cos( _cnt[i], _cnt[(i+1) % 4], _cnt[(i+2) % 4] ) for i in xrange(4)])
                squares.append(cnt)
                self.draw_label(img, "RECT", cnt)
	if len(cnt) < 3 or len(cnt) > 6:
	        area = cv2.contourArea(cnt)
	        x,y,w,h = cv2.boundingRect(cnt)
	        radius = w / 2
	        #print abs(1 - (1.0 * w /  h)), w, h
	        #print abs(1 - (area / (math.pi * radius * radius)))
		if h > 0 and w > 0:	        
		    if abs(1 - (1.0 * w /  h)) < 0.5 and abs(1 - (area / (math.pi * radius * radius))) < 0.5:
		        circles.append(cnt)
			self.draw_label(img, "CIRCLE", cnt)
    if len(squares) > 0:     
	cv2.drawContours( img, squares, -1, (0, 255, 0), 3 )	
    if len(circles) > 0:
     	cv2.drawContours( img, circles, -1, (0, 0, 255), 3 )	
    return img

  def updatePosition(self, topic):
    self.currentPoseX = topic.pose.position.x 
    self.currentPoseY = topic.pose.position.y
    self.currentPoseZ = topic.pose.position.z
        

  def callback(self,data):
    global saveFolder
    global nframe
    global InitialX, InitialY
    #saveFolder = '/home/buti/images/uav3/'
    try:
      cvImage = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
    nframe = nframe + 1
    image2Analyse = cvImage.copy()    
    #imgfile = saveFolder + ("%05d" % nframe) + '.png'
    img = self.detect_obstacles(image2Analyse)
    label = "POS" +  "(" + ("%.2fm" % self.currentPoseX) + "," + ("%.2fm" % self.currentPoseY) + "," + ("%.2fm" % self.currentPoseZ) + ")"
    if InitialX == 0 and InitialY == 0:
	InitialX = self.currentPoseX
	InitialY = self.currentPoseY
    cv2.putText(img,label, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #cv2.imwrite(imgfile, img) 
    small = cv2.resize(img, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow("Tracker", small)
    cv2.waitKey(10);
    
    if len(self.obstacles.objects) >= 5:
	self.objects_pub.publish(self.obstacles)

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
 
