#!/usr/bin/env python

## Randa Almadhoun
## 20-Feb-2017


import rosbag

import sys, struct, time, os
import math
#import pygmaps 
import datetime
from tf.msg import tfMessage
from argparse import ArgumentParser
import matplotlib.pyplot as plt
from geometry_msgs.msg import Quaternion
from numpy import mean, array, hypot, diff, convolve, arange, sin, cos, ones, pi, matrix
from tf.transformations import euler_from_quaternion,quaternion_from_euler,quaternion_multiply,quaternion_matrix
import tf
import codecs
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import webbrowser

def plot3(a,b,c,mark="o",col="r"):
  from matplotlib import pyplot
  import pylab
  from mpl_toolkits.mplot3d import Axes3D
  pylab.ion()
  fig = pylab.figure()
  ax = Axes3D(fig)
  ax.scatter(a, b, c,marker=mark,color=col)
  pylab.show()
  
try:
  import gmplot
except ImportError:
  print("This script requires gmplot be available to plot gps on the map.")
  print("On Ubuntu: pip install gmplot --user")
  exit(1)

parser = ArgumentParser(description='Plot GPS data')
parser.add_argument('bag', metavar='FILE', type=str, help='input bag file')
args = parser.parse_args()

bag = rosbag.Bag(args.bag)


# Read gps points and other information from the bag file
gps_x = []
gps_y = []
gps_z = []
time_t = []
flags = []
numberSat = []
distance = []
for topic, msg, time in bag.read_messages(topics=("/mavros/gps_reroute/gps_fix", "/mavros/gps_reroute/gps_fix")):
  #TODO IMPORTANT NOTE: flip the longitude and latitude if you are using the old logs 
        gps_x.append(msg.latitude/10000000) 
        gps_y.append(msg.longitude/10000000) 
        gps_z.append(msg.height/1000)
	time_t.append(msg.header.stamp.secs)
	flags.append(msg.flag)
	numberSat.append(msg.numOfSat)
	distance.append(msg.distance)
print( "The number of points found in /mavros/gps_reroute/gps_fix is : " + str(len(gps_x)) )

gps_global_x = []
gps_global_y = []
gps_global_z = []
for topic, msg, time in bag.read_messages(topics=("/mavros/global_position/global", "/mavros/global_position/global")):
  #TODO IMPORTANT NOTE: flip the longitude and latitude if you are using the old logs 
        gps_global_x.append(msg.latitude) 
        gps_global_y.append(msg.longitude) 
        gps_global_z.append(msg.altitude)

print( "The number of points found in /mavros/global_position/global : " + str(len(gps_x)) )




#plot GPS on the GOOGLE MAP using gmplot 
gmap = gmplot.GoogleMapPlotter(gps_x[0], gps_y[0], 16)
gmap.plot(gps_x, gps_y, 'red', edge_width=10)
#gmap.scatter(gps_x, gps_y, 'r', marker=True) ## to add markers on the gps points in the map (it will appear in red)
gmap.draw("mymap1.html")
webbrowser.open_new_tab('mymap1.html')

#plot gps global on the GOOGLE MAP using gmplot
gmap1 = gmplot.GoogleMapPlotter(gps_global_x[0], gps_global_y[0], 16)
gmap1.plot(gps_global_x, gps_global_y, 'purple', edge_width=10)
#gmap.scatter(gps_global_x, gps_global_y, 'k', marker=True) ## to add markers on the gps points in the map (it will appear in black)
gmap1.draw("mymap2.html")
webbrowser.open_new_tab('mymap2.html')


#plot flags, number of satellite, distance, latitde , longitude, altitude vs time
plt.figure(1)
plt.xlabel('time (secs)')
plt.ylabel('flag ')
plt.plot(time_t,flags,'-g',label='flag')
plt.legend(loc='lower right')
plt.grid(True)

plt.figure(2)
plt.xlabel('time (secs)')
plt.ylabel('number of satellites ')
plt.plot(time_t,numberSat,'-g',label='Number of satellites')
plt.legend(loc='lower right')
plt.grid(True)

plt.figure(3)
plt.xlabel('time (secs)')
plt.ylabel('distance ')
plt.plot(time_t,distance,'-g',label='Distance')
plt.legend(loc='lower right')
plt.grid(True)

plt.figure(4)
plt.xlabel('time (secs)')
plt.ylabel('latitiude (deg)')
plt.plot(time_t,gps_x,'-g',label='latitiude')
plt.legend(loc='lower right')
plt.grid(True)

plt.figure(5)
plt.xlabel('time (secs)')
plt.ylabel('longitude (deg)')
plt.plot(time_t,gps_y,'-g',label='longitude')
plt.legend(loc='lower right')
plt.grid(True)

plt.figure(6)
plt.xlabel('time (secs)')
plt.ylabel('altitude (deg)')
plt.plot(time_t,gps_z,'-g',label='altitude')
plt.legend(loc='lower right')
plt.grid(True)

plt.show()
exit(1)