import rospy
import smach
import math

from actionlib import *
from actionlib.msg import *
from scitos_ptu.msg import PtuGotoAction,PtuGotoGoal

from std_srvs.srv import Empty
from geometry_msgs.msg import TransformStamped
from util import get_ros_service
from sensor_msgs.msg import JointState, PointCloud2
from rosbag_openni_compression.msg import RosbagRecordCameraAction, RosbagRecordCameraActionGoal, RosbagRecordCameraGoal

class StartRecording(smach.State):
    def __init__(self, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'success'],
                              input_keys=['sweep_folder'],
                              output_keys=['sweep_folder'])

        self._data_compression_action = actionlib.SimpleActionClient(
            '/rosbag_record_camera',
            RosbagRecordCameraAction)
        rospy.loginfo("Initialising LeanObjects SM: "
                      "Waiting for data compression action server...")
        if not self._data_compression_action.wait_for_server(rospy.Duration(5)):
            rospy.logerr("Data compression action can't be found!")
            raise Exception("LearnObject SM can't initialise; missing service.")
        self._data_compression_goal = RosbagRecordCameraGoal()

        self._debug_mode = debug_mode
        self._debug_services = debug_services


    def execute(self, userdata):
        try:
            self._data_compression_goal.camera = 'head_xtion'
            self._data_compression_goal.bagfile = userdata['sweep_folder']+'tracking.bag'
            self._data_compression_goal.with_depth = True
            self._data_compression_goal.with_rgb = True

            # Start logging of camera topics
            self._data_compression_action.send_goal(self._data_compression_goal)
            rospy.sleep(20)
            rospy.loginfo("Starting recording head_xtion camera topics ")
            return "success"
        except Exception, e:
            print e
            return "error"

class StopRecording(smach.State):
    def __init__(self, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'success'],
                              input_keys=['sweep_folder'],
                              output_keys=['sweep_folder'])

        self._data_compression_action = actionlib.SimpleActionClient(
            '/rosbag_record_camera',
            RosbagRecordCameraAction)
        rospy.loginfo("Initialising LeanObjects SM: "
                      "Waiting for data compression action server...")
        if not self._data_compression_action.wait_for_server(rospy.Duration(5)):
            rospy.logerr("Data compression action can't be found!")
            raise Exception("LearnObject SM can't initialise; missing service.")

        self._debug_mode = debug_mode
        self._debug_services = debug_services


    def execute(self, userdata):
        try:
            self._data_compression_action.cancel_all_goals()
            rospy.loginfo("Stopped recording head_xtion topics")
            return "success"
        except:
            return "error"
