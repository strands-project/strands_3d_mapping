#! /usr/bin/env python

import roslib; 
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

import scitos_ptu.msg
from sensor_msgs.msg import *
from std_msgs.msg import String
from cloud_merge.msg import *
import threading
import math
import time


class SweepServer:
  def __init__(self):
    rospy.init_node('sweep_action_server')
    self.log_pub = rospy.Publisher('/ptu/log', String)

    self.server = actionlib.SimpleActionServer('do_sweep', cloud_merge.msg.SweepAction, self.execute, False)
    self.server.register_preempt_callback(self.preemptCallback)
    self.server.start()
    rospy.loginfo('Sweep action server started')
 
    self.aborted = False
    self.preempted = False
    self.preempt_timeout = 0.3 # seconds
    self.sweep_timeout = 240 # seconds


    self.feedback = scitos_ptu.msg.PanTiltFeedback()
    self.result = scitos_ptu.msg.PanTiltResult()
    self.result.success = True

    self.preempt_lock = threading.Lock()


    self.ptugoal = scitos_ptu.msg.PanTiltGoal()

    self.client = actionlib.SimpleActionClient("ptu_pan_tilt_metric_map", scitos_ptu.msg.PanTiltAction)
    self.client.wait_for_server()
    rospy.loginfo('ptu_pan_tilt_metric_map client created')

  def execute(self, goal):
    legal_sweep = False
    
    if goal.type == "complete" :
        rospy.loginfo('Complete sweep type: -160 20 160 -30 30 30')
        self.ptugoal.pan_start = -160
        self.ptugoal.pan_step = 20
        self.ptugoal.pan_end = 160
        self.ptugoal.tilt_start = -30
        self.ptugoal.tilt_step = 30
        self.ptugoal.tilt_end = 30
        legal_sweep = True
    elif goal.type == 'medium':
        rospy.loginfo('Medium sweep type: -160 20 160 -30 -30 -30')
        self.ptugoal.pan_start = -160
        self.ptugoal.pan_step = 20
        self.ptugoal.pan_end = 160
        self.ptugoal.tilt_start = -30
        self.ptugoal.tilt_step = -30
        self.ptugoal.tilt_end = -30
        legal_sweep = True
    elif goal.type == 'short':
        rospy.loginfo('Short sweep type: -160 40 160 -30 -30 -30')
        self.ptugoal.pan_start = -160
        self.ptugoal.pan_step = 40
        self.ptugoal.pan_end = 160
        self.ptugoal.tilt_start = -30
        self.ptugoal.tilt_step = -30
        self.ptugoal.tilt_end = -30
        legal_sweep = True
    elif goal.type == 'shortest':
        rospy.loginfo('Shortest sweep type: -140 60 160 -30 -30 -30')
        self.ptugoal.pan_start = -160
        self.ptugoal.pan_step = 60
        self.ptugoal.pan_end = 140
        self.ptugoal.tilt_start = -30
        self.ptugoal.tilt_step = -30
        self.ptugoal.tilt_end = -30
        legal_sweep = True
    else:
        rospy.loginfo('Unknown sweep type')

    self.preempted = False
    self.aborted = False
    if legal_sweep:
        self._doSweep()

    if self._get_preempt_status():
		self.result.success = False
		self.server.set_preempted(self.result)

    elif self.aborted:
		self.result.success = False
		self.server.set_aborted(self.result)
    elif legal_sweep:
        if self.client.get_state() == GoalStatus.SUCCEEDED :
            self.result.success = True
            self.server.set_succeeded(self.result)
        else :
            self.result.success = False
            self.server.set_succeeded(self.result)
    else:
        self.result.success = False
        self.server.set_succeeded(self.result)


  def _doSweep(self):
        time_waited = 0
        self.client.send_goal(self.ptugoal)
        self.client.wait_for_result(rospy.Duration(self.preempt_timeout))
        status= self.client.get_state()
        while not status == GoalStatus.SUCCEEDED and not time_waited > self.sweep_timeout and not self._get_preempt_status():
                # figure out how to get the feedback out of the client
#                feedback = self.client.get_feedback()
#                self.server.publish_feedback(feedback)
                time_waited += self.preempt_timeout
                self.client.wait_for_result(rospy.Duration(self.preempt_timeout))
                status= self.client.get_state()

        if self._get_preempt_status():
                # this action server has been preempted; preempt the other one as well
                self.client.cancel_goal()
        elif time_waited > self.sweep_timeout or status != GoalStatus.SUCCEEDED:
                # didn't manage to reach the PTU position
                self.client.cancel_goal()
                self.aborted = True

  def _get_preempt_status(self):
	self.preempt_lock.acquire()
	preempted = self.preempted
	self.preempt_lock.release()
	return preempted	

  def preemptCallback(self):
    self.preempt_lock.acquire()
    self.preempted = True
    self.preempt_lock.release()


if __name__ == '__main__':
  server = SweepServer()
  rospy.spin()
