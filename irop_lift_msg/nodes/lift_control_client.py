#!/usr/bin/env python

import rospy
import time

from irop_lift.msg import RelayControl


def talker():
    pub_lift = rospy.Publisher("lift/control", RelayControl, queue_size=100)
    rospy.init_node("lift_control_client")
    
    r = rospy.Rate(10)
    msg = RelayControl()
    while not rospy.is_shutdown():
        msg.direction = 0
        pub_lift.publish(msg)
        r.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



