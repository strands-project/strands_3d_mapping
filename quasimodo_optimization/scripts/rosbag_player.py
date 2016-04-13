#! /usr/bin/env python

import roslib; roslib.load_manifest('quasimodo_optimization')
import rospy
import subprocess
import signal
import os

import actionlib

import quasimodo_optimization.msg

class RosbagPlayerServer(object):
    # create messages that are used to publish feedback/result
    #_feedback = topic_logger.msg.topicLoggerFeedback()
    #_result = topic_logger.msg.topicLoggerResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, quasimodo_optimization.msg.rosbag_playAction, execute_cb = self.execute_cb)
        self._as.start()
        rospy.loginfo('Server is up')

    def execute_cb(self, goal):
        # decide whether recording should be started or stopped
        if goal.command == "start":
            #start to record the topics
            rospy.loginfo('now the topic recording should start')
            args = " --rate=5"
            args = args + " " + goal.file
            command = "rosbag play" + args
            self.p = subprocess.Popen("exec " + command, stdin=subprocess.PIPE, preexec_fn=os.setsid, shell=True)
            rospy.loginfo(self.p.pid)

            # check if the goal is preempted
            rate = rospy.Rate(1.0)
            while not rospy.is_shutdown() and self.p.poll() is None:
                #self.p.communicate(input=b'\n')
                if self._as.is_preempt_requested():
                    rospy.loginfo('Logging is preempted')
                    os.killpg(os.getpgid(self.p.pid), signal.SIGINT)
                    self._as.set_preempted()
                    break
                rate.sleep()

        elif goal.command == "stop":
            #stop to record the topics
            rospy.loginfo('now the topic playing should stop')
            rospy.loginfo(self.p.pid)
            os.killpg(os.getpgid(self.p.pid), signal.SIGINT)

            rospy.loginfo("I'm done")

        else:
            rospy.loginfo('goal.command is not valid')

if __name__ == '__main__':
    rospy.init_node('rosbag_player')
    RosbagPlayerServer(rospy.get_name())
    rospy.spin()
