#!/usr/bin/env python

from gripper.srv import *
import rospy
import serial
import serial.tools.list_ports
ports = list(serial.tools.list_ports.comports())
for p in ports:
    print p
ser = serial.Serial(p[0], 9600)    
def ActuateGripperCallback(req):
    if req.OnOff== True:
        commandtosend="1"
        gripperstatus="on"
    else:
        commandtosend="2"
        gripperstatus="off"
    print "Sending to serial port [%s] to turn the gripper [%s]"%(commandtosend, gripperstatus)
    ser.write(str(commandtosend))
    return AttachResponse(req.OnOff)

def gripper_server():
    rospy.init_node('GripperActuationServer')
    s = rospy.Service('GripperOnOFF', Attach, ActuateGripperCallback)
    print "Ready to Actuate Gripper"
    rospy.spin()

if __name__ == "__main__":
    gripper_server()
