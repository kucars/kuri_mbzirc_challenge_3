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
#CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Python libs
#roslib.load_manifest('mbzirc_challenge3_image_analysis')
import sys
import os
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

VERBOSE=False

threshold = 0.40
kernel5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(20,20))
x_co = 0
y_co = 0
hsv = None
H = 0
S = 0
V = 0
thr_H = 180*threshold
thr_S = 255*threshold
thr_V = 255*threshold

tracking_timeout = 10 ##Seconds until tracking lost

image_rescale = 2.5

VidOutput = True

ObjectList = []

class object_tracking:

    def __init__(self, actionServer, goal):
    #def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        #self.image_pub = rospy.Publisher("/output/image_raw/compressed",
        #    CompressedImage)
        self.bridge = CvBridge()
        self.actionServer = actionServer
        self.obstacles = Objects()
        # subscribed Topic
        #self.subscriber = rospy.Subscriber("/zed_camera/rgb/image_rect_color/compressed", CompressedImage, self.callback,  queue_size = 1)
        #self.left = rospy.Subscriber("/zed_camera/left/image_rect_color/compressed", CompressedImage, self.callback_left,  queue_size = 1)
        #self.right = rospy.Subscriber("/zed_camera/right/image_rect_color/compressed", CompressedImage, self.callback_right,  queue_size = 1)
        #self.right = rospy.Subscriber("/image_raw/compressed", CompressedImage, self.callback_right,  queue_size = 1)
        #self.right = rospy.Subscriber("/uav_3/downward_cam/camera/image",Image,self.callback_right)
        self.image_sub = message_filters.Subscriber('/uav_'+str(goal.uav_id)+'/downward_cam/camera/image', Image)
        self.info_sub = message_filters.Subscriber('/uav_'+str(goal.uav_id)+'/downward_cam/camera/camera_info', CameraInfo)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub], 10)
        self.ts.registerCallback(self.callback_right)
        
        mavros.set_namespace('/uav_3/mavros')
        self.currentPoseX = -1
        self.currentPoseY = -1
        self.currentPoseZ = -1
        self.sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), SP.PoseStamped, self.updatePosition)
        
        self.image = None
        self.left_image = None
        self.right_image = None
        #self.data_small = np.genfromtxt('../config/small.dat', np.float32, delimiter=',')
        #self.data_big = np.genfromtxt('../config/big.dat', np.float32, delimiter=',')
        rospack = rospkg.RosPack()
        packagePath = rospack.get_path('kuri_object_tracking')
        print packagePath
        self.data_small = np.genfromtxt(packagePath+'/config/small.dat', np.float32, delimiter=',')
        self.data_big = np.genfromtxt(packagePath+'/config/big.dat', np.float32, delimiter=',')
        self.samples = np.reshape(self.data_small, (-1, 7))
        self.samples_big = np.reshape(self.data_big, (-1, 7))
        self.objects_count = 0

	#self.video = cv2.VideoWriter('detection.avi', cv2.cv.CV_FOURCC(*'XVID'), 3, (1280,720))
        #if VidOutput == True:
        #    self.video = cv2.VideoWriter("tester.avi", cv2.cv.CV_FOURCC('M','J','P','G'), 10.0,(1228, 1027),True)
        #print self.samples
        #self.objects_hist = []
        #self.objects_roi = []         
        
        self.edgeThresholdSize = 0.1
        self.width = 1600
        self.height = 1200
    
    def updatePosition(self, topic):
        self.pose = PoseWithCovariance()
        self.pose.pose = topic.pose
        self.currentPoseX = topic.pose.position.x 
        self.currentPoseY = topic.pose.position.y
        self.currentPoseZ = topic.pose.position.z    
        #print 'Updating Pos', self.currentPoseX, self.currentPoseY, self.currentPoseZ
    
    def process(self):
        if self.image == None or self.left_image == None or self.right_image == None:
            return
        
        # disparity settings
        window_size = 1
        min_disp = 32
        num_disp = 112-min_disp
        stereo = cv2.StereoSGBM(
        minDisparity = min_disp,
        numDisparities = num_disp,
        SADWindowSize = window_size,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32,
        disp12MaxDiff = 1,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        fullDP = False
        )
        
        
        disparity = stereo.compute(self.left_image,self.right_image).astype(np.float32)
        disparity = (disparity-min_disp)/num_disp
        cv2.imshow('left', self.left_image)
        cv2.imshow('right', self.right_image)
        cv2.imshow('disparity', disparity)
        self.image = None
        self.left_image = None
        self.right_image = None

    def callback_left(self, ros_data):
        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        self.left_image = image_np;
        cv2.imshow('left', image_np)
        #self.process()
        cv2.waitKey(2)
        
    def redFilter(self, img):
        #H = 12
        #S = 161
        #V = 255
        ##SIMULATOR VALUES
        H = 0
        S = 255
        V = 135        
        
        src = cv2.blur(img, (10,10))
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        min_color = np.array([H-thr_H,S-thr_S,V-thr_V])
        max_color = np.array([H+thr_H,S+thr_S,V+thr_V])
        mask = cv2.inRange(hsv, min_color, max_color)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5)        
        return mask

    def blueFilter(self, img):
        #H = 112
        #S = 170
        #V = 251
        ##SIMULATOR VALUES
        H = 120
        S = 177
        V = 156
        
        src = cv2.blur(img, (10,10))
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        min_color = np.array([H-thr_H,S-thr_S,V-thr_V])
        max_color = np.array([H+thr_H,S+thr_S,V+thr_V])
        mask = cv2.inRange(hsv, min_color, max_color)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5)        
        return mask
        
    def greenFilter(self, img):
        #H = 72 #69
        #S = 211 #208
        #V = 246 #255
        
        ##SIMULATOR VALUES
        H = 60
        S = 255
        V = 177
        
        src = cv2.blur(img, (10,10))
        hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
        min_color = np.array([H-thr_H,S-thr_S,V-thr_V])
        max_color = np.array([H+thr_H,S+thr_S,V+thr_V])
        mask = cv2.inRange(hsv, min_color, max_color)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel5)        
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
        #p[0] = self.currentPoseX - p[0]
        #p[1] = self.currentPoseY - p[1]
        #p[2] = self.currentPoseZ - p[2]
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
            if self.actionServer.hasGoal:
                self.actionServer.update(obstacle.getAsObject())
            else:
                self.obstacles = Objects()
                for obj in ObjectList:
                    self.obstacles.append(obj.getAsObject())
                self.actionServer.objects = obstacles
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
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
                if dst < 300 and dst >= 0:
                    obj.update(min_cnt, min_pixel)
                    coords = self.imageToWorld(obj.cx,obj.cy)
                    obj.wx = coords[0]
                    obj.wy = coords[1]
                    obj.wz = coords[2]
                    cv2.putText(self.right_image, '#'+str(obj.object_id)+'|pos:'+str(coords[0])+','+str(coords[1]), (obj.cx,obj.cy), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,0,0), thickness = 1)
        for cnt in contours:
	    x,y,w,h = cv2.boundingRect(cnt)
	    if w < 15 or h < 15:
		continue
	    if w > 100 or h > 100:
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
               		
#               		if c == 'blue':
#                   		c = 'BLUE'
#                   		cv2.putText(self.right_image,c, (x+w,y), cv2.FONT_HERSHEY_PLAIN, 1.0, (255,0,0), thickness = 2)
#               		if c == 'green':
#                       		c = 'GREEN'
#                       		cv2.putText(self.right_image,c, (x+w,y), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,255,0), thickness = 2)
#               		if c == 'red':
#                       		c = 'RED'
#                       		cv2.putText(self.right_image,c, (x+w,y), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,0,255), thickness = 2)
#               		cv2.drawContours(self.right_image, [cnt], 0, (255,255,255), 2)
	
            
    def detect_big_objects(self, img):
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
                
	
        

    def callback_right(self, ros_data, camera_info):
        #print camera_info
        self.img_proc = image_geometry.PinholeCameraModel()
        self.img_proc.fromCameraInfo(camera_info)
        #n = self.img_proc.projectPixelTo3dRay((900,30))
        #print n
        #### direct conversion to CV2 ####
        #np_arr = np.fromstring(ros_data.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        cvImage = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
        image_np = cvImage.copy()
        image_np = cv2.resize(image_np, (0,0), fx=image_rescale, fy=image_rescale) 
        self.right_image = image_np;
        #self.track_objects()
        red_mask = self.redFilter(image_np)
        blue_mask = self.blueFilter(image_np)
        black_mask = self.greenFilter(image_np)
        red = cv2.bitwise_and(image_np,image_np,mask = red_mask)
        blue = cv2.bitwise_and(image_np,image_np,mask = blue_mask)
        black = cv2.bitwise_and(image_np,image_np,mask = black_mask)
        filtered = cv2.bitwise_or(red, blue)
        filtered = cv2.bitwise_or(filtered, black)
        self.detect_objects(filtered, 'all')
        #self.detect_objects(red, 'red')
        #self.detect_objects(blue, 'blue')
        #self.detect_objects(black, 'green')
        self.detect_big_objects(red)
        #self.process()
        #cv2.imshow('filtered', filtered)
        
        cv2.waitKey(2)    
        if VidOutput == True:    
            label = "POS" +  "(" + ("%.2fm" % self.currentPoseX) + "," + ("%.2fm" % self.currentPoseY) + "," + ("%.2fm" % self.currentPoseZ) + ")"
            cv2.putText(self.right_image,label, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('object_tracking', self.right_image)
            
        #cv2.imwrite('right.png',image_np)   
        #if self.objects_count > 0:
            #os._exit(1)
        ##Update list to remove lost objects
        if ObjectList.count > 0:
            ObjectList[:] = [x for x in ObjectList if not x.islost(tracking_timeout)]

        
    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        self.image = image_np;
        cv2.imshow('cv_img', image_np)
        #self.process()
        cv2.waitKey(2)
        #### Feature detectors using CV2 #### 
        # "","Grid","Pyramid" + 
        # "FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF"
        #method = "GridFAST"
        #feat_det = cv2.FeatureDetector_create(method)
        #time1 = time.time()

        # convert np image to grayscale
#        featPoints = feat_det.detect(
#            cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
#        time2 = time.time()
#        if VERBOSE :
#            print '%s detector found: %s points in: %s sec.'%(method,
#                len(featPoints),time2-time1)
#
#        for featpoint in featPoints:
#            x,y = featpoint.pt
#            cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
#        
#        cv2.imshow('cv_img', image_np)
#        cv2.waitKey(2)

        
        #self.subscriber.unregister()

#def main(args):
#    '''Initializes and cleanup ros node'''
#    ic = rosbag_detector()
#    rospy.init_node('image_feature', anonymous=True)
#    try:
#        rospy.spin()
#    except KeyboardInterrupt:
#        print "Shutting down ROS Image feature detector module"
#    cv2.destroyAllWindows()
#
#if __name__ == '__main__':
#    main(sys.argv)
