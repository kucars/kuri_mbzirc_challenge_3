# -*- coding: utf-8 -*-
"""
Created on Wed Jun 22 07:24:40 2016

@author: buti
"""

import cv2
import numpy as np
from math import *

##Tracker
import cv
import numpy
from utils import cv2array, array2img
from tracker import skl, warpImg, maxLikelihood
from polygon import Polygon

class object_tracker:
  def __init__(self, x,y,w,h, id):
    self.data = None
    self.U = None
    self.D = None
    self.mu = None
    self.n = None
    self.i = 0
    center = (x, y)
    size = (w, h)
    outSize = (32, 32)
    self.polygon = Polygon(center, size, outSize=outSize)
    self.id = id

    
  def init_track_object(self, x,y,w,h):
    center = (x, y)
    size = (w, h)
    outSize = (32, 32)
    self.polygon = Polygon(center, size, outSize=outSize)

  def track_object(self, inImgRGB):
    outSize = (32, 32)
    #SKL initialization
    actualisation = 10
    
    data = self.data
    U = self.U
    D = self.D
    mu = self.mu
    n = self.n
    ff = 0.99
    K = 16
    #i = 0
    polygon = self.polygon

    # Dumb particles filter parameters
    num = 200
    sigmaCenterX = 20
    sigmaCenterY = 20
    sigmaSizeX = 0#5
    sigmaSizeY = 0#3
    sigmaRot = 0#10 * numpy.pi / 180.0
    sigmaIncl = 0#1 * numpy.pi / 180.0
    bitmap = cv.CreateImageHeader((inImgRGB.shape[1], inImgRGB.shape[0]), cv.IPL_DEPTH_8U, 3)
    cv.SetData(bitmap, inImgRGB.tostring(), inImgRGB.dtype.itemsize * 3 * inImgRGB.shape[1])
    inImgGray = cv.CreateImage(cv.GetSize(bitmap), bitmap.depth, 1)
    cv.CvtColor(bitmap, inImgGray, cv.CV_RGB2GRAY)
    #inImgGray = cv2.cvtColor(inImgRGB, cv2.COLOR_BGR2GRAY)
    polygonTmp = []
    polygonTmp.append(polygon)
    subImgs = None
    for k in range(num):
        centerTmp = polygon.center[0] + numpy.random.randn() * sigmaCenterX, polygon.center[1] + numpy.random.randn() * sigmaCenterY
        sizeTmp = polygon.size[0] + numpy.random.randn() * sigmaSizeX, polygon.size[1] + numpy.random.randn() * sigmaSizeY
        rotationTmp = polygon.rotation + numpy.random.randn() * sigmaRot
        inclinaisonTmp = polygon.transvection + numpy.random.randn() * sigmaIncl
        polygonTmp.append(Polygon(centerTmp, sizeTmp, rotationTmp, inclinaisonTmp, outSize))

        subImgTmp = warpImg(inImgGray, polygonTmp[k])
        if subImgs == None:
            subImgs = numpy.mat(cv2array(subImgTmp)).ravel().T
        else:
            subImgs = numpy.hstack((subImgs, numpy.mat(cv2array(subImgTmp)).ravel().T))
    idx = maxLikelihood(subImgs, U, D, mu)
    polygon = polygonTmp[idx]

    # SKL!!!
    newData = subImgs[:, idx]
    if data == None:
        data = newData
    else:
        data = numpy.hstack((data, newData))
    if self.i % actualisation == 0:
        U, D, mu, n = skl(data=data, U0=U, D0=D, mu0=mu, n0=n, ff=ff, K=K)
        data = None
    #for k in range(num):
	#if k % 10 == 0:
		#cv.PolyLine(bitmap, (polygonTmp[k].corners(),), True, cv.RGB(255, 0, 0))
    #cv.ShowImage("Warped Stream", array2img(subImgs[:, idx], outSize))
    #cv.ShowImage("Mean Stream", array2img(mu, outSize))
    #cv.ShowImage("1st Eigenface Stream", array2img(U[:, 0], outSize))
    self.i = self.i + 1
    self.data = data
    self.U = U
    self.D = D
    self.mu = mu
    self.n = n
    # Draw the extraction polygon
    #cv.PolyLine(bitmap, (polygon.corners(),), True, cv.RGB(0, 255, 0))
    #cv.ShowImage("Webcam Stream", bitmap)
    self.polygon = polygon
    return polygon