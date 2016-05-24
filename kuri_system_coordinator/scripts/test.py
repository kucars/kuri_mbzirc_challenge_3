#!/usr/bin/env python
print "Test"
from kuri_msgs.msg import *

obj = Object()
objs = Objects()

obj.width  = 100
obj.height = 200

objs.objects.append(obj)

print obj
print objs