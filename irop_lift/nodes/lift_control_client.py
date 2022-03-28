#!/usr/bin/env python

import rospy
import actionlib

from irop_lift.msg import lift_controlAction, lift_controlGoal

def call_server(direction):
    client = actionlib.SimpleActionClient(
        'lift_control_server', lift_controlAction
    )
    client.wait_for_server()

    goal = lift_controlGoal()
    goal.direction = direction

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:         
        rospy.loginfo("CALL DONE !!!!")

    result = client.get_result()
    rospy.loginfo(result)


def main():
    print('\n *** Lift Control Client ***')
    print('''Choose function
    1 : Lift Up
    2 : Lift Half Up
    3 : Lift Down
    4 : Lift Half Down
    =======================
    ''')

    choose = input('press: ')

    if choose == 1:
        rospy.loginfo("Lift Up")
        call_server("up")
    elif choose == 2:
        rospy.loginfo("Lift Half Up")
        call_server("hup")
    elif choose == 3:   
        rospy.loginfo("Lift Down")
        call_server("down")
    elif choose == 4:
        rospy.loginfo("Lift Half Down")
        call_server("hdown")
    else:
        pass
    
    choose = 0

if __name__ == "__main__":
    rospy.init_node("lift_control_client")

    while True:
        main()

    # rospy.spin()