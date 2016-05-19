#!/usr/bin/env python


import rospy
from std_msgs.msg import String
import random
import numpy
import yaml
#from matplotlib.pyplot import figure, show
import matplotlib.pyplot as plt
import ast
import geometry_msgs
from kuri_msgs.msg import *
from geometry_msgs.msg import Pose
from mavros import setpoint as SP


rospy.init_node('tester_task', anonymous=False)



n = Object()
ns = Objects()

n.color = 'red'
n.header = ''
n.pose = ''
n.width = 100
n.height = 200
n.velocity = 10

a = ObjectsMap
ns.objects.append(n)

a.Object = ns

while not rospy.is_shutdown():
    pub = rospy.Publisher('tester_task', ObjectsMap, queue_size=10)
    pub.publish(a)
