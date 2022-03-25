#!/usr/bin/env python

# topic
# https://github.com/fkie/catkin_lint/discussions/97

import os, sys
from ctypes import CDLL, c_int
import rospy

from irop_lift.msg import RelayControl

def resource_path(relative_path):
    """ Get absolute path to resource, works for dev and for PyInstaller """
    base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

relay_lib = CDLL(resource_path("../libs/usb_relay_device.so"))
init = relay_lib.usb_relay_init()
device_enum = relay_lib.usb_relay_device_enumerate()
device = str(relay_lib.usb_relay_device_open(str(device_enum)))

def lift_control_callback(relay):
    global relay_lib, device
    rospy.loginfo("direction: " + str(relay.direction))
    if relay.direction == 0:
        rospy.loginfo("UP")
        channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(2))
        channel = relay_lib.usb_relay_device_open_one_relay_channel(device, c_int(1))
    elif relay.direction == 1:
        rospy.loginfo("DOWN")
        channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(1))
        channel = relay_lib.usb_relay_device_open_one_relay_channel(device, c_int(2))   
    elif relay.direction == 2:
        rospy.loginfo("STOP")
        channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(1))
        channel = relay_lib.usb_relay_device_close_one_relay_channel(device, c_int(2))



if __name__ == "__main__":
    rospy.init_node("lift_control_server")

    sub_lift = rospy.Subscriber("lift/control", RelayControl, lift_control_callback)
    
    rospy.spin()