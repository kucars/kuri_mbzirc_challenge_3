#!/usr/bin/env python
#!/usr/bin/env python
#
# Copyright 2016 KURI KUSTAR .
# Author Tarek Taha : tarek.taha@kustar.ac.ae
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
print "Navigator Test"
from kuri_msgs.msg import *
import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as gm
import tf.transformations
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from math import *

def send_task():
    pub = rospy.Publisher('kuri_msgs/NavTasks', NavTasks, queue_size=1,latch=True)
    rospy.init_node('uav1_task_allocater', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    obj = Object()
    objs = Objects()
    obj.width  = 100
    obj.height = 200
    obj.pose.pose.position = gm.Point(0,0,10.0)
    objs.objects.append(obj)

    navtasks = NavTasks()
    navtask  = NavTask()
    path     = nav_msgs.Path()

    pose = gm.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = 'map'
    pose.pose.position = gm.Point(0,0,0)
    pose.pose.orientation = gm.Quaternion(*quaternion_from_euler(0, 0, 0))
    path.poses.append(pose)

    navtask.path     = path
    navtask.uav_id   = 1
    navtask.uav_name = 'UAV1'
    navtask.object = obj

    navtasks.tasks.append(navtask)
    pub.publish(navtasks)
    print "Task Published"
    while not rospy.is_shutdown():
        #yaw_degrees = 0  # North
        #yaw = radians(yaw_degrees)
        #quaternion = quaternion_from_euler(0, 0, yaw)
        #msg.pose.orientation = SP.Quaternion(*quaternion)
        #rospy.loginfo(msg.pose)
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_task()
    except rospy.ROSInterruptException:
        pass