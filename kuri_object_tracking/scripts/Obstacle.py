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

import cv2
import numpy as np
import time
from kuri_msgs.msg import *
from geometry_msgs.msg import *

class Obstacle:
  def __init__(self, object_id, contour, color, pixel):
      self.object_id = object_id
      self.contour = contour
      self.color = color
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
      self.roiBox = (tl[0], tl[1], br[0], br[1])
      self.pixel = pixel
      self.cx = x+w/2
      self.cy = y+h/2
      self.width = w
      self.height = h
      self.timestamp = time.time()
      self.wx = 0
      self.wy = 0
      self.wz = 0
      
  def update(self, contour, pixel):
      self.contour = contour
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
      self.roiBox = (tl[0], tl[1], br[0], br[1])
      self.pixel = pixel
      self.cx = x+w/2
      self.cy = y+h/2
      self.width = w
      self.height = h
      self.timestamp = time.time()
      
      
  def islost(self, timeout):
      t = time.time() - self.timestamp
      if t > timeout:
          return True
      return False
      
  def getAsObject(self):
      o = Object()
      print o
      o.pose.pose.position.x = self.wx
      o.pose.pose.position.y = self.wy
      o.pose.pose.position.z = self.wz
      o.velocity = Twist() ##TODO
      o.width = self.width
      o.height = self.height
      o.color = self.color
      return o