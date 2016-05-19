#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from mavros import setpoint as SP
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from math import *

def send_task():
    pub = rospy.Publisher('uav1_target_location', SP.PoseStamped, queue_size=1)
    rospy.init_node('uav1_task_allocater', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = SP.PoseStamped(
            header=SP.Header(
                frame_id="base_footprint",  # no matter, plugin don't use TF
                stamp=rospy.Time.now()),    # stamp should update
    )
    while not rospy.is_shutdown():
        msg.pose.position.x = 10
        msg.pose.position.y = 10
        msg.pose.position.z = 10
		#(21.9855, 65.0235, 0.515313)
        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation = SP.Quaternion(*quaternion)
        rospy.loginfo(msg.pose)
        pub.publish(msg)
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_task()
    except rospy.ROSInterruptException:
        pass
