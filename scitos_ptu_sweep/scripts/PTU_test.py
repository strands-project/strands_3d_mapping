#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import scitos_ptu_sweep.msg


def ptu_sweep_client():
    
    client = actionlib.SimpleActionClient('PTUSweep', scitos_ptu_sweep.msg.PTUSweepAction)
    
    client.wait_for_server()
    rospy.loginfo(" ... Init done")

    ptugoal = scitos_ptu_sweep.msg.PTUSweepGoal()

    ptugoal.max_pan = float(sys.argv[1])
    ptugoal.max_tilt = float(sys.argv[2])
    ptugoal.min_pan = float(sys.argv[3])
    ptugoal.min_tilt = float(sys.argv[4])
    ptugoal.pan_step = float(sys.argv[5])
    ptugoal.tilt_step = float(sys.argv[6])

    # Sends the goal to the action server.
    client.send_goal(ptugoal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    print 'Argument List:',str(sys.argv)
    if len(sys.argv) < 7 :
	sys.exit(2)
    rospy.init_node('ptu_sweep_test_py')
    ps = ptu_sweep_client()
    print ps
