#! /usr/bin/env python
import rospy
import actionlib
from learn_objects_action.msg import LearnObjectAction, LearnObjectGoal

def learn_object(waypoint):
    client = actionlib.SimpleActionClient('learn_object', LearnObjectAction)

    client.wait_for_server()
    goal = LearnObjectGoal(waypoint=waypoint)
    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result() 

if __name__ == '__main__':
    try:
        rospy.init_node('learn_object_client')
        result = learn_object("WayPoint9")
        print "Result:", ', ', result
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
