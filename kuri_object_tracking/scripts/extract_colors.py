#!/usr/bin/env python
"""
Created on Sat Mar 11 18:25:56 2017

@author: buti
"""

# import the necessary packages
import numpy as np
import argparse
import cv2
import operator

color_thresh = 256/2
 
def histogram(image):
	# initialize the list of histograms, one calculated
	# for each channel of the image
	hists = []

	# loop over the image channels
	for chan in cv2.split(image):
		# create a histogram for the current channel and
		hist = cv2.calcHist([chan], [0], None, [256], [0, 256])
		hist = cv2.normalize(hist)
		hists.append(hist)

	# return the list of histograms
	return hists 
 
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True, help = "Path to the image")
args = vars(ap.parse_args())

# load the image, clone it for output, and then convert it to grayscale
image = cv2.imread(args["image"])
output = image.copy()
#ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCR_CB)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# detect circles in the image
circles = cv2.HoughCircles(gray, cv2.cv.CV_HOUGH_GRADIENT, 1.5, 100)

H_red =  0
S_red = 255
V_red = 135
H_blue = 120
S_blue = 177
V_blue = 156
H_green = 60
S_green = 255
V_green = 177
H_yellow = 31
S_yellow = 167
V_yellow = 228
f = open('colors.yaml', 'w')
# ensure at least some circles were found
if circles is not None:
	# convert the (x, y) coordinates and radius of the circles to integers
	circles = np.round(circles[0, :]).astype("int")
 
	# loop over the (x, y) coordinates and radius of the circles
	for (x, y, r) in circles:
		# draw the circle in the output image, then draw a rectangle
		# corresponding to the center of the circle
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
  
            roiPts = []
            roiPts.append([x, y])
            roiPts.append([x, y+r])
            roiPts.append([x+r, y])
            roiPts.append([x+r, y+r])
            roiPts = np.array(roiPts)
            s = roiPts.sum(axis = 1)
            tl = roiPts[np.argmin(s)]
            br = roiPts[np.argmax(s)]
            roi = image[tl[1]:br[1], tl[0]:br[0]]
            hist = histogram(roi)
            R, max_value = max(enumerate(hist[2]), key=operator.itemgetter(1))
            G, max_value = max(enumerate(hist[1]), key=operator.itemgetter(1))
            B, max_value = max(enumerate(hist[0]), key=operator.itemgetter(1))
            label = 'R:'+ str(R) + ' G:'+ str(G) + ' B:' + str(B)
            cv2.putText(output,label, (x - r,y - r/2), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            roi = hsv[tl[1]:br[1], tl[0]:br[0]]
            hist = histogram(roi)
            H, max_value = max(enumerate(hist[0]), key=operator.itemgetter(1))
            S, max_value = max(enumerate(hist[1]), key=operator.itemgetter(1))
            V, max_value = max(enumerate(hist[2]), key=operator.itemgetter(1))
            label = 'H:'+ str(H) + ' S:'+ str(S) + ' V:' + str(V)
            cv2.putText(output,label, (x - r,y), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            color = 'unknown'
            if R > color_thresh and G < color_thresh and B < color_thresh:
                color = 'RED'
                H_red = H
                S_red = S
                V_red = V
            if B > color_thresh and G < color_thresh and R < color_thresh:
                color = 'BLUE'
                H_blue = H
                S_blue = S
                V_blue = V
            if G > color_thresh and R < color_thresh and B < color_thresh:
                color = 'GREEN'
                H_green = H
                S_green = S
                V_green = V
            if R > color_thresh and G > color_thresh and B < color_thresh:
                color = 'YELLOW'
                H_yellow = H
                S_yellow = S
                V_yellow = V
            cv2.putText(output,color, (x - r,y + r/2), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
            #print 'H:', H, 'S:', S, 'V:', V 
 
	# show the output image
	cv2.imshow("output", np.hstack([image, output]))
	cv2.waitKey(0)
	f.write('H_red: ' + str(H_red) + '\n')
	f.write('S_red: ' + str(S_red) + '\n')
 	f.write('V_red: ' + str(V_red) + '\n')
  	f.write('H_blue: ' + str(H_blue) + '\n')
	f.write('S_blue: ' + str(S_blue) + '\n')
 	f.write('V_blue: ' + str(V_blue) + '\n')
  	f.write('H_green: ' + str(H_green) + '\n')
	f.write('S_green: ' + str(S_green) + '\n')
 	f.write('V_green: ' + str(V_green) + '\n')
  	f.write('H_yellow: ' + str(H_yellow) + '\n')
	f.write('S_yellow: ' + str(S_yellow) + '\n')
 	f.write('V_yellow: ' + str(V_yellow) + '\n')
	f.close()
 