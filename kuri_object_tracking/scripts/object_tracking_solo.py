#! /usr/bin/env python
#Copyright (c) 2016, Buti Al Delail, Tarek Taha
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

# Python libs
#roslib.load_manifest('mbzirc_challenge3_image_analysis')
import sys
import os
import time
import math
# numpy and scipy
import numpy as np
# OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# Ros libraries
import roslib
import rospy
import message_filters
import image_geometry
import mavros
from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
# Ros Messages
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from kuri_msgs.msg import *
from geometry_msgs.msg import *
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError
from Obstacle import Obstacle
import rospkg
import yaml
#Test Dropzone
import thread
import threading
from kuri_object_tracking.srv import *

VERBOSE=False

threshold = 0.40
kernel5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(20,20))
x_co = 0
y_co = 0
hsv = None

thr_H = 180*threshold
thr_S = 255*threshold
thr_V = 255*threshold

tracking_timeout = 5 ##Seconds until tracking lost

image_rescale = 1

VidOutput = True
TestDropZone = True

ObjectList = []

##Detection Interval, run detection every N frames
interval_small = 3
interval_big = 4

config_yaml = None

class object_tracking:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospack = rospkg.RosPack()
        packagePath = rospack.get_path('kuri_object_tracking')
        with open(packagePath+'/config/simulator.yaml', 'r') as stream:
            try:
                config_yaml  = yaml.load(stream)
                self.H_red   = config_yaml.get('H_red')
                self.S_red   = config_yaml.get('S_red')
                self.V_red   = config_yaml.get('V_red')
                self.H_blue  = config_yaml.get('H_blue')
                self.S_blue  = config_yaml.get('S_blue')
                self.V_blue  = config_yaml.get('V_blue')
                self.H_green = config_yaml.get('H_green')
                self.S_green = config_yaml.get('S_green')
                self.V_green = config_yaml.get('V_green')
                self.uav_id  = config_yaml.get('uav_id')
            except yaml.YAMLError as exc:
                print(exc)
        self.colors2Track = 'none'
        self.navigate_started = False
        self.bridge    = CvBridge()
        self.obstacles = Objects()
        self.image_sub = message_filters.Subscriber('/uav_'+str(self.uav_id)+'/downward_cam/camera/image', Image)
        self.info_sub  = message_filters.Subscriber('/uav_'+str(self.uav_id)+'/downward_cam/camera/camera_info', CameraInfo)
        #self.outpub = rospy.Publisher('/uav_'+str(self.uav_id)+'/downward_cam/camera/detection_image',Image, queue_size=100, latch=True)
        self.objectPub = rospy.Publisher('/uav_'+str(self.uav_id)+'/tracked_object/object', Object, queue_size=10)

        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 10)
        self.ts.registerCallback(self.callback)

        self.controllerService = rospy.Service('trackObjectService', Object2Track , self.object2TrackCallback)
        mavros.set_namespace('/uav_'+str(self.uav_id)+'/mavros')
        self.currentPoseX = -1
        self.currentPoseY = -1
        self.currentPoseZ = -1
        self.pub = SP.get_pub_position_local(queue_size=10)
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.updatePosition)
        
        self.image = None
        self.left_image = None
        self.right_image = None

        self.data_small = np.genfromtxt(packagePath+'/config/small.dat', np.float32, delimiter=',')
        self.data_big = np.genfromtxt(packagePath+'/config/big.dat', np.float32, delimiter=',')
        self.data_dropzone = np.genfromtxt(packagePath+'/config/dropzone.dat', np.float32, delimiter=',')
        self.samples = np.reshape(self.data_small, (-1, 7))
        self.samples_big = np.reshape(self.data_big, (-1, 7))
        self.samples_dropzone = np.reshape(self.data_dropzone, (-1, 7))
        self.objects_count = 0
        self.frame = 0
        self.new_objects = Objects()
        self.new_objects_count = 0
        self.navigating = False
        self.done = False       
        
        self.edgeThresholdSize = 0.1
        self.width = 1600
        self.height = 1200

    def object2TrackCallback(self,req):
        self.colors2Track = req.color
        reply = Object2TrackResponse()
        reply.accepted = True
        print "Received an object"
        return reply

    def updatePosition(self, topic):
        self.pose = PoseWithCovariance()
        self.pose.pose = topic.pose
        self.currentPoseX = topic.pose.position.x 
        self.currentPoseY = topic.pose.position.y
        self.currentPoseZ = topic.pose.position.z         
        
    def redFilter(self, img):
        #H = 12
        #S = 161
        #V = 255
        ##SIMULATOR VALUES
        #H = 0
        #S = 255
        #V = 135        
        
        src = cv2.blur(img, (10,10))
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        min_color = np.array([self.H_red-thr_H,self.S_red-thr_S,self.V_red-thr_V])
        max_color = np.array([self.H_red+thr_H,self.S_red+thr_S,self.V_red+thr_V])
        mask = cv2.inRange(hsv, min_color, max_color)
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5)
        return mask

    def blueFilter(self, img):
        #H = 112
        #S = 170
        #V = 251
        ##SIMULATOR VALUES
        #H = 120
        #S = 177
        #V = 156
        
        src = cv2.blur(img, (10,10))
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        min_color = np.array([self.H_blue-thr_H,self.S_blue-thr_S,self.V_blue-thr_V])
        max_color = np.array([self.H_blue+thr_H,self.S_blue+thr_S,self.V_blue+thr_V])
        mask = cv2.inRange(hsv, min_color, max_color)
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5)
        return mask
        
    def greenFilter(self, img):
        #H = 72 #69
        #S = 211 #208
        #V = 246 #255
        
        ##SIMULATOR VALUES
        #H = 60
        #S = 255
        #V = 177
        
        src = cv2.blur(img, (10,10))
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        min_color = np.array([self.H_green-thr_H,self.S_green-thr_S,self.V_green-thr_V])
        max_color = np.array([self.H_green+thr_H,self.S_green+thr_S,self.V_green+thr_V])
        mask = cv2.inRange(hsv, min_color, max_color)
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5)
        return mask
        
    def whiteFilter(self, img):
        #H = 72 #69
        #S = 211 #208
        #V = 246 #255
        
        ##SIMULATOR VALUES
        self.H_white = 0
        self.S_white = 0
        self.V_white = 255
        
        src = cv2.blur(img, (10,10))
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        min_color = np.array([self.H_white-thr_H,self.S_white-thr_S,self.V_white-thr_V])
        max_color = np.array([self.H_white+thr_H,self.S_white+thr_S,self.V_white+thr_V])
        mask = cv2.inRange(hsv, min_color, max_color)
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5)
        return mask        
        
    def overlap(self, a, b):  # returns None if rectangles don't intersect
        dx = min(a[2], b[2]) - max(a[0], b[0])
        dy = min(a[3], b[3]) - max(a[1], b[1])
        if (dx>=0) and (dy>=0):
            return dx*dy
        return 0
        
    def nearEdge(self, a):
      if a[0] < self.width*self.edgeThresholdSize or a[1] < self.height*self.edgeThresholdSize or a[0] > (self.width - self.width*self.edgeThresholdSize) or a[1] > (self.height - self.height*self.edgeThresholdSize):
          return True
      return False      
      
    def imageToWorld(self, x, y):
        x = x / image_rescale
        y = y / image_rescale
        p = self.img_proc.projectPixelTo3dRay((x,y)) 
        p = np.array(p)
        return p
       

    def object_detected(self, contour, color):
        x,y,w,h = cv2.boundingRect(contour)
        roiPts = []
        roiPts.append([x, y])
        roiPts.append([x, y+h])
        roiPts.append([x+w, y])
        roiPts.append([x+w, y+h])
        roiPts = np.array(roiPts)
        s = roiPts.sum(axis = 1)
        tl = roiPts[np.argmin(s)]
        br = roiPts[np.argmax(s)]
        roiBox = (tl[0], tl[1], br[0], br[1])
        cX = x + (w/2)
        cY = y + (h/2)
        #print cX, cY
        pixel = self.right_image[cY, cX]
        overlap = 0
        for obj in ObjectList:
            overlap = overlap + self.overlap(obj.roiBox, roiBox)
        if overlap == 0:
            obstacle = Obstacle(self.objects_count, contour, color, pixel)
            ObjectList.append(obstacle)
            self.objects_count = self.objects_count + 1
#    def object_detected(self, x, y, w, h):
        #if self.objects_count > 1:
        #    return        
#        return        
#        self.objects_count = self.objects_count + 1
#        orig = self.right_image.copy()
#        roiPts = []
#        roiPts.append([x, y])
#        roiPts.append([x, y+h])
#        roiPts.append([x+w, y])
#        roiPts.append([x+w, y+h])
#        roiPts = np.array(roiPts)
#        s = roiPts.sum(axis = 1)
#        tl = roiPts[np.argmin(s)]
#        br = roiPts[np.argmax(s)]
#        roi = orig[tl[1]:br[1], tl[0]:br[0]]
#        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
#        roiHist = cv2.calcHist([roi], [0], None, [16], [0, 180])
#        #roiHist = cv2.normalize(roiHist, roiHist, 0, 255, cv2.NORM_MINMAX)
#        roiBox = (tl[0], tl[1], br[0], br[1])
#        print 'ROI', roiBox
#        overlap = 0
#        for roi in self.objects_roi:
#            overlap = overlap + self.overlap(roi, roiBox)
#        if overlap == 0:
#            cv2.imwrite('roi.png',roi)   
#            cv2.imshow('roi', roi)
#            print 'New Object', x, y, w, h
#            self.objects_hist.append(roiHist)
#            self.objects_roi.append(roiBox)
#        else:
#            print 'Overlapping' , overlap
            
        
        
#    def track_objects(self):
#        termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 80, 1)
#        hsv = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2HSV)
#        i = 0
#        for roiHist in self.objects_hist:
#            roiBox = self.objects_roi[i]
#            backProj = cv2.calcBackProject([hsv], [0], roiHist, [0, 180], 1)
#            #(r, roiBox) = cv2.CamShift(backProj, roiBox, termination)
#            #pts = np.int0(cv2.cv.BoxPoints(r))
#            #cv2.polylines(self.right_image, [pts], True, (0, 255, 0), 2)
#            track_window = (roiBox[0], roiBox[1], roiBox[2]-roiBox[0], roiBox[3]-roiBox[1])
#            ret, track_window = cv2.meanShift(backProj, track_window, termination)
#            x,y,w,h = track_window
#            cv2.rectangle(self.right_image, (x,y), (x+w,y+h), 10, 2)
#            i = i + 1
        
        
    def detect_objects(self, img, c):
        if c == 'none':
          return
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1 )
        #m1 = [[  0.790115  ], [  3.16737453], [  4.66246158], [  7.23488413], [-13.65758756], [ -9.15648336], [ 13.2095227 ]]            

        for obj in ObjectList:   
            if obj.color == c or c == 'all':
                a = cv2.HuMoments(cv2.moments(obj.contour))
                b = -np.sign(a)*np.log10(np.abs(a))
                b = np.array([b[0][0], b[1][0], b[2][0], b[3][0], b[4][0], b[5][0], b[6][0], obj.cx, obj.cy, obj.pixel[0], obj.pixel[1], obj.pixel[2]])
                dst = float("inf")
                min_cnt = None
                min_pixel = None
                for cnt in contours:
                    x,y,w,h = cv2.boundingRect(cnt)
                    cx = x+w/2
                    cy = y+h/2
                    pixel = img[cy, cx]
                    a = cv2.HuMoments(cv2.moments(cnt))
                    m = -np.sign(a)*np.log10(np.abs(a))
                    m = np.array([m[0][0], m[1][0], m[2][0], m[3][0], m[4][0], m[5][0], m[6][0], cx, cy, pixel[0], pixel[1], pixel[2]])
                    d = sum(abs(m - b))
                    if d < dst:
                        dst = d
                        min_cnt = cnt
                        min_pixel = pixel
                #print 'MIN_DST', dst
                if dst < 300 and dst >= 0 or (obj.color == 'DROP_ZONE'):
                    obj.update(min_cnt, min_pixel)
                    coords = self.imageToWorld(obj.cx,obj.cy)
                    obj.wx = self.currentPoseX + coords[0]
                    obj.wy = self.currentPoseY - coords[1]
                    obj.wz = self.currentPoseZ - coords[2]
                    obj.camx = coords[0]
                    obj.camy = coords[1]
                    obj.camz = coords[2]
                    cv2.putText(self.right_image, '#'+str(obj.object_id)+'|pos:'+str(coords[0])+','+str(coords[1]), (obj.cx,obj.cy), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,0,0), thickness = 1)        
        
        if self.frame % interval_small:
            return        
        
        for cnt in contours:
	    x,y,w,h = cv2.boundingRect(cnt)
	    if w < 15 or h < 15:
		continue
	    if w > 100 or h > 100:
		continue
            a = cv2.HuMoments(cv2.moments(cnt))
            m = -np.sign(a)*np.log10(np.abs(a))
            m = [m[0][0], m[1][0], m[2][0], m[3][0], m[4][0], m[5][0], m[6][0]]
	    cX = x + (w/2)
	    cY = y + (h/2)
	    #print cX, cY
	    color = img[cY, cX]
	    dst = float("inf")
	    for sample in self.samples:
		d = sum(abs(m - sample))
		if d < dst:
			dst = d
	    #coords = self.imageToWorld(cX, cY)
	    #cv2.putText(self.right_image,"dst:"+str(dst.astype(int))+'|pos:'+str(coords[0])+','+str(coords[1]), (x,y-10), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,0,0), thickness = 1)
	    if dst < 5:
               		B = color[0]
               		G = color[1]
               		R = color[2]
               		if G >= R and G >= B and R >= G and R >= B:
                            c = 'YELLOW';
                            cv2.putText(self.right_image,c, (x+w,y), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,255,255), thickness = 2)
               		elif B >= G and B >= R:
                   		c = 'BLUE';
                   		cv2.putText(self.right_image,c, (x+w,y), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,0,0), thickness = 2)
               		elif G >= R and G >= B:
                       		c = 'GREEN';
                       		cv2.putText(self.right_image,c, (x+w,y), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,255,0), thickness = 2)
               		elif R >= G and R >= B:
                       		c = 'RED';
                       		cv2.putText(self.right_image,c, (x+w,y), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,0,255), thickness = 2)
               		self.object_detected(cnt, c)
               		cv2.drawContours(self.right_image, [cnt], 0, (255,255,255), 2)

            
    def detect_big_objects(self, img):        
        if self.frame % interval_big:
            return
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1 )

        for cnt in contours:
	    x,y,w,h = cv2.boundingRect(cnt)
	    if w < 100 and h < 100:
		continue
            a = cv2.HuMoments(cv2.moments(cnt))
            m = -np.sign(a)*np.log10(np.abs(a))
            m = [m[0][0], m[1][0], m[2][0], m[3][0], m[4][0], m[5][0], m[6][0]]
	    M = cv2.moments(cnt)
	    cX = x + (w/2)
	    cY = y + (h/2)
	    #print cX, cY
	    color = img[cY, cX]
            dst = float("inf")
	    for sample in self.samples_big:
		d = sum(abs(m - sample))
		if d < dst:
			dst = d
            
            cv2.putText(self.right_image,"dst:"+str(dst.astype(int)), (x,y-10), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,255,255), thickness = 1)
            #print dst        
            if dst < 15:
                cv2.putText(self.right_image,"BIG: ORANGE", (x+w,y), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,255,255), thickness = 2)
                self.object_detected(cnt, 'ORANGE')
                cv2.drawContours(self.right_image, [cnt], 0, (255,255,255), 2)
                
	
    def detect_drop_zone(self, img):     
        for obj in ObjectList:##Already have drop zone
            if obj.color == 'DROP_ZONE' and obj.islost(timeout) == False:
                return
        #if self.frame % interval_big:
         #   return
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1 )

        for cnt in contours:
	    x,y,w,h = cv2.boundingRect(cnt)
	    if w < 100 and h < 100:
		continue
	    if w > 600 and h > 600:
		continue
            a = cv2.HuMoments(cv2.moments(cnt))
            m = -np.sign(a)*np.log10(np.abs(a))
            m = [m[0][0], m[1][0], m[2][0], m[3][0], m[4][0], m[5][0], m[6][0]]
	    M = cv2.moments(cnt)
	    cX = x + (w/2)
	    cY = y + (h/2)
	    #print cX, cY
	    color = img[cY, cX]
            dst = float("inf")
	    for sample in self.samples_dropzone:
		d = sum(abs(m - sample))
		if d < dst:
			dst = d
            
            cv2.putText(self.right_image,"dst:"+str(dst.astype(int)), (x,y-10), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,255,255), thickness = 1)
            #print dst        
            if dst < 45:
                cv2.putText(self.right_image,"DROP_ZONE", (x+w,y), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,255,0), thickness = 2)
                self.object_detected(cnt, 'DROP_ZONE')
                cv2.drawContours(self.right_image, [cnt], 0, (255,0,0), 2)

    def callback(self, ros_data, camera_info):
        #return
        #print camera_info
        self.new_objects = Objects()
        self.new_objects_count = 0
        self.img_proc = image_geometry.PinholeCameraModel() ##need calibration?
        self.img_proc.fromCameraInfo(camera_info)
        #n = self.img_proc.projectPixelTo3dRay((900,30))
        #print n
        #### direct conversion to CV2 ####
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        cvImage = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        image_np = cvImage.copy()
        #image_np = cv2.resize(image_np, (0,0), fx=0.5, fy=0.5) 
        #self.track_objects()
        red_mask = self.redFilter(image_np)
        blue_mask = self.blueFilter(image_np)
        black_mask = self.greenFilter(image_np)
        white_mask = self.whiteFilter(image_np)
        red      = cv2.bitwise_and(image_np,image_np,mask = red_mask)
        blue     = cv2.bitwise_and(image_np,image_np,mask = blue_mask)
        black    = cv2.bitwise_and(image_np,image_np,mask = black_mask)
        white    = cv2.bitwise_and(image_np,image_np,mask = white_mask)
        filtered = cv2.bitwise_or(red, blue)
        filtered = cv2.bitwise_or(filtered, black)
        filtered = cv2.bitwise_or(filtered, white)
        
        filtered = cv2.resize(filtered, (0,0), fx=image_rescale, fy=image_rescale) 
        red = cv2.resize(red, (0,0), fx=image_rescale, fy=image_rescale) 
        white = cv2.resize(white, (0,0), fx=image_rescale, fy=image_rescale) 
        image_np = cv2.resize(image_np, (0,0), fx=image_rescale, fy=image_rescale) 
        self.right_image = image_np;        
        
        self.detect_objects(filtered, self.colors2Track)
        #self.detect_big_objects(red)
        #self.detect_drop_zone(white)

        self.frame = self.frame + 1

        if VidOutput == True:    
            cv2.waitKey(2)    
            #label = "POS" +  "(" + ("%.2fm" % self.currentPoseX) + "," + ("%.2fm" % self.currentPoseY) + "," + ("%.2fm" % self.currentPoseZ) + ")"
            #cv2.putText(self.right_image,label, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            disp = cv2.resize(self.right_image, (0,0), fx=1/image_rescale, fy=1/image_rescale) 
            #disp = self.right_image 
            #self.outpub.publish(self.bridge.cv2_to_imgmsg(disp, "bgr8"))
            cv2.imshow('object_tracking', disp)
            
        #cv2.imwrite('right.png',image_np)   
        #if self.objects_count > 0:
            #os._exit(1)
            
        ##Publish new Objects
        #if self.actionServer.hasGoal and self.new_objects_count > 0:
        #    self.actionServer.update(self.new_objects
        ##Update list to remove lost objects
        if ObjectList.count > 0:
            ObjectList[:] = [x for x in ObjectList if not x.islost(tracking_timeout)]
        self.obstacles = Objects()
        height, width, channels = filtered.shape
        imgCenterX = width/2.0
        imgCenterY = height/2.0
        closestDist = 10000
        obj2Send = None
        for obj in ObjectList:
            self.obstacles.objects.append(obj.getAsObject())
            distX2Center = imgCenterX - obj.cx
            distY2Center = imgCenterY - obj.cy
            dist = math.sqrt(distX2Center*distX2Center + distY2Center*distY2Center)
            if dist < closestDist:
              closestDist = dist
              obj2Send = obj.getAsObject()
        if obj2Send:
          print obj2Send.pose.pose.position.x,  obj2Send.pose.pose.position.y
          self.objectPub.publish(obj2Send)
def main(args):
    '''Initializes and cleanup ros node'''
    objectTracking = object_tracking()
    rospy.init_node('object_tracking_solo', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down object tracking"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
