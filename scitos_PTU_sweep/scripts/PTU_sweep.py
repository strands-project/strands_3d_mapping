#!/usr/bin/env python

import rospy
import actionlib
import flir_pantilt_d46.msg
from sensor_msgs.msg import JointState
import scitos_PTU_sweep.msg

class PTUSweep():
    
    # Create feedback and result messages
    _feedback = scitos_PTU_sweep.msg.PTUSweepFeedback()
    _result   = scitos_PTU_sweep.msg.PTUSweepResult()

    def __init__(self, name):
        rospy.loginfo("Starting %s", name)
        self._action_name = name
        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, scitos_PTU_sweep.msg.PTUSweepAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")
        #sub_topic = rospy.get_param("~pedestrian_locations", '/pedestrian_localisation/localisations')
        #rospy.Subscriber(sub_topic, PedestrianLocations, self.pedestrianCallback, None, 5)
        self.client = actionlib.SimpleActionClient('SetPTUState', flir_pantilt_d46.msg.PtuGotoAction)
        self.client.wait_for_server()
        rospy.loginfo(" ... Init done")

    def executeCallback(self, goal):   
        current_pan=goal.min_pan
        ptugoal = flir_pantilt_d46.msg.PtuGotoGoal()
        ptugoal.pan_vel = 42
        ptugoal.tilt_vel = 21
        while current_pan < goal.max_pan:
            current_tilt = goal.min_tilt
            self._feedback.current_pan=current_pan
            while current_tilt < goal.max_tilt:
                print current_pan, current_tilt
                
                ptugoal.pan = current_pan
                ptugoal.tilt = current_tilt
                self.client.send_goal(ptugoal)
                print ptugoal
                self.client.wait_for_result()
                print self.client.get_result()
                self._feedback.current_tilt=current_tilt
                current_tilt += goal.tilt_step     
                self._as.publish_feedback(self._feedback)               
            current_pan += goal.pan_step
        self._result.success = True
        self._as.set_succeeded(self._result)


    def preemptCallback(self):
        #ptu_command = JointState()
        #ptu_command.name=["pan", "tilt"]
        #ptu_command.position=[0,0]
        #ptu_command.velocity=[1.0]
        #self.pub.publish(ptu_command)
        self._result.success = False
        self._as.set_preempted(self._result)


if __name__ == '__main__':
    rospy.init_node("PTUSweep")
    ps = PTUSweep(rospy.get_name())
    rospy.spin()
