# -*- coding: utf-8 -*-
"""
Created on Wed Jun 22 07:14:19 2016

@author: buti
"""

import cv2
import numpy as np
from math import *
from pylab import *


class object_detector:
  def __init__(self):
      self.background_threshold = 155
        
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

  def detect_obstacles(self, img):
    obstacles = []
    nobg = self.remove_background(img, self.background_threshold)
    gray = cv2.cvtColor(nobg,cv2.COLOR_BGR2GRAY)
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
	cnt_len = cv2.arcLength(cnt, True)
        cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
	if len(cnt) >= 4 and len(cnt) <= 6 and cv2.contourArea(cnt) > 50 and cv2.isContourConvex(cnt):
             rect = cv2.boundingRect(cnt)
             M = cv2.moments(cnt)
             x = int(M['m10']/M['m00'])
             y = int(M['m01']/M['m00'])
             obstacles.append([x, y, rect[2] * 1.5, rect[3] * 1.5])
	if len(cnt) < 3 or len(cnt) > 6:
	        area = cv2.contourArea(cnt)
	        x,y,w,h = cv2.boundingRect(cnt)
	        radius = w/2.0
		if h > 0 and w > 0 and radius > 0:	        
		    if abs(1 - (1.0 * w /  h)) < 0.5 and abs(1 - (area / (math.pi * radius * radius))) < 0.5:
                      rect = cv2.boundingRect(cnt)
                      M = cv2.moments(cnt)
                      x = int(M['m10']/M['m00'])
                      y = int(M['m01']/M['m00'])
                      obstacles.append([x, y, rect[2] * 1.5, rect[3] * 1.5])
    return obstacles        