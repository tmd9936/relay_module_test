#!/usr/bin/env python

import rospy
import os, sys
from ctypes import c_int
import ctypes
import actionlib

# from std_msgs.msg import Int16
from irop_lift.msg import lift_controlAction, lift_controlFeedback, lift_controlResult

RELAY_LIB_PATH = "../libs/usb_relay_device.so"
HALF_TIME = 13
NO_STOP_TIME = 0

def resource_path(relative_path):
    """ Get absolute path to resource"""
    base_path = getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

def exc(msg):  
    rospy.logerr(msg)
    return Exception(msg)

def fail(msg): raise exc(msg)

global device

class L: pass   # Global object for the DLL
setattr(L, "dll", None)
L.dll = ctypes.CDLL(resource_path(RELAY_LIB_PATH))

usb_relay_lib_funcs = [
  # TYpes: h=handle (pointer sized), p=pointer, i=int, e=error num (int), s=string
  ("usb_relay_device_enumerate",               'h', None),
  ("usb_relay_device_close",                   'e', 'h'),
  ("usb_relay_device_open_with_serial_number", 'h', 'si'),
  ("usb_relay_device_get_num_relays",          'i', 'h'),
  ("usb_relay_device_get_id_string",           's', 'h'),
  ("usb_relay_device_next_dev",                'h', 'h'),
  ("usb_relay_device_get_status_bitmap",       'i', 'h'),
  ("usb_relay_device_open_one_relay_channel",  'e', 'hi'),
  ("usb_relay_device_close_one_relay_channel", 'e', 'hi'),
  ("usb_relay_device_close_all_relay_channel", 'e', None)
  ]

ret = L.dll.usb_relay_init()
if ret != 0 : rospy.logerr("Failed lib init!")

"""
Tweak imported C functions
This is required in 64-bit mode. Optional for 32-bit (pointer size=int size)
Functions that return and receive ints or void work without specifying types.
"""
ctypemap = { 'e': ctypes.c_int, 'h':ctypes.c_void_p, 'p': ctypes.c_void_p,
        'i': ctypes.c_int, 's': ctypes.c_char_p}
for x in usb_relay_lib_funcs :
    fname, ret, param = x

    try:
        f = getattr(L.dll, fname)
    except Exception:  
        fail("Missing lib export:" + fname)
    ps = []
    if param :
        for p in param :
            ps.append( ctypemap[p] )
    f.restype = ctypemap[ret]
    if len(ps) > 0:
        f.argtypes = ps
    setattr(L, fname, f)

def Lift_up(time):
    global device
    channel = L.usb_relay_device_close_one_relay_channel(device, c_int(2))
    rospy.loginfo(channel)
    channel = L.usb_relay_device_open_one_relay_channel(device, c_int(1))
    rospy.loginfo(channel)

    if time != NO_STOP_TIME:
        rospy.loginfo("LIFT HALF UP")
        rospy.sleep(int(time))
        channel = L.usb_relay_device_close_one_relay_channel(device, c_int(1))
    else:
        rospy.loginfo("LIFT UP")
    
def Lift_down(time):
    global device
    channel = L.usb_relay_device_close_one_relay_channel(device, c_int(1))
    rospy.loginfo(channel)
    channel = L.usb_relay_device_open_one_relay_channel(device, c_int(2))
    rospy.loginfo(channel)

    if time != NO_STOP_TIME:
        rospy.loginfo("LIFT HALF DOWN")
        rospy.sleep(int(time))
        channel = L.usb_relay_device_close_one_relay_channel(device, c_int(2))
    else:
        rospy.loginfo("LIFT DOWN")

class Action_Server():
    def __init__(self):
        self.lift_server = actionlib.SimpleActionServer(
            'lift_control_server', lift_controlAction, 
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.lift_server.start()

    def execute_cb(self, goal):
        global L, device
        feedback = lift_controlFeedback()
        result_a = lift_controlResult()

        feedback_str = ''
        result_str = ''
        rate = rospy.Rate(20)

        try:
            enuminfo = L.usb_relay_device_enumerate()
            idstrp = L.usb_relay_device_get_id_string(enuminfo)
            device = L.usb_relay_device_open_with_serial_number(idstrp, 5)

            if goal.direction == "up":
                feedback_str = "upppp"
                feedback.feedback_key = feedback_str

                result_str = "LIFT UP DONE!"
                result_a.result_key = result_str

                self.lift_server.publish_feedback(feedback)
                Lift_up(NO_STOP_TIME)

            elif goal.direction == "hup":
                feedback_str = "half upppp"
                feedback.feedback_key = feedback_str

                result_str = "LIFT HALF UP DONE!"
                result_a.result_key = result_str

                self.lift_server.publish_feedback(feedback)
                Lift_up(HALF_TIME)

            elif goal.direction == "down":
                feedback_str = "downnnn"
                feedback.feedback_key = feedback_str

                result_str = "LIFT DOWN DONE!"
                result_a.result_key = result_str

                self.lift_server.publish_feedback(feedback)            
                Lift_down(NO_STOP_TIME)

            elif goal.direction == "hdown":
                feedback_str = "half downnnn"
                feedback.feedback_key = feedback_str

                result_str = "LIFT HALF DOWN DONE!"
                result_a.result_key = result_str

                self.lift_server.publish_feedback(feedback)            
                Lift_down(HALF_TIME)

            else:
                pass

            self.lift_server.set_succeeded(result_a)
        except Exception:  
            fail("fail call enum or device:")
        finally:
            device_close = L.usb_relay_device_close(enuminfo)
            # realy_exit = L.usb_relay_exit()


if __name__ == "__main__":
    rospy.init_node("lift_control_server")
    rospy.loginfo("Begin Lift Control Server")

    s = Action_Server()
    rospy.spin()
    # device_close = L.usb_relay_device_close(enuminfo)
    # realy_exit = L.usb_relay_exit()