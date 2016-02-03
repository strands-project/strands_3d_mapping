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
from static_transform_manager.srv import SetTransformation, StopTransformation
from ptu_follow_frame.srv import StartFollowing

class TurnPTUToObject(smach.State):
    def __init__(self, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'success', 'preempted'],
                              input_keys=['dynamic_object'],
                              output_keys=['dynamic_object'])

        self._ptu_angle_pub = rospy.Publisher("/ptu/cmd", JointState)
        self._jnts = JointState()
        self._jnts.name=["pan","tilt"]
        self._jnts.velocity=[1,1]
        self._jnts.position=[0,0]
        self.ptu_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
        rospy.loginfo("Wait for PTU action server")
        self.ptu_client.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Done")
        self._debug_mode = debug_mode
        self._debug_services = debug_services


    def execute(self, userdata):
        try:
            if self._debug_mode:
                self._debug_services.set_status("Turn PTU to cluster?")
                proceed = self._debug_services.get_proceed(("OK",))
                if proceed == "PREEMPT":
                    return 'preempted'
            # start transformation
            self._jnts.position=[math.radians(userdata.dynamic_object.pan_angle),
                                 math.radians(userdata.dynamic_object.tilt_angle)]
            goal = PtuGotoGoal()
            goal.pan = userdata.dynamic_object.pan_angle
            goal.tilt = userdata.dynamic_object.tilt_angle
            goal.pan_vel = 30
            goal.tilt_vel = 30
            rospy.loginfo("SET PTU: pan: %f tilt: %f", goal.pan, goal.tilt)
            self.ptu_client.send_goal(goal)
            self.ptu_client.wait_for_result()
            rospy.loginfo("Reached ptu goal")

            print "Capturing a new shot of that object before tracking."
            # So uggly force get through publisher queue
            for i in range(30):
                userdata.dynamic_object.object_cloud= rospy.wait_for_message("/head_xtion/depth_registered/points", PointCloud2)
                #  is already there
            print "ok."
            return "success"
        except Exception, e:
            print e
            return "error"

class StartTransformation(smach.State):
    def __init__(self, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'success'],
                              input_keys=['dynamic_object_centroid'])

        self._set_transform = get_ros_service("/static_transforms_manager/set_tf",
                                              SetTransformation)
        self._debug_mode = debug_mode
        self._debug_services = debug_services


    def execute(self, userdata):
        try:
            # stop transformation
            trans = TransformStamped()

            trans.transform.translation.x = userdata.dynamic_object_centroid.x
            trans.transform.translation.y = userdata.dynamic_object_centroid.y
            trans.transform.translation.z = userdata.dynamic_object_centroid.z
            trans.transform.rotation.w = 1
            trans.header.frame_id = "map"
            trans.child_frame_id = "cluster"
            self._set_transform(trans)
            return "success"
        except Exception, e:
            print e
            return "error"

class StopSendingTransformation(smach.State):
    def __init__(self, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'success'],
                              input_keys=['dynamic_object_centroid'])
        self._stop_transform = get_ros_service("/static_transforms_manager/stop_tf",
                                               StopTransformation)
        self._debug_mode = debug_mode
        self._debug_services = debug_services


    def execute(self, userdata):
        try:
            # stop transformation
            self._stop_transform("cluster")
            return "success"
        except Exception, e:
            print e
            return "error"

class StartPTUTrack(smach.State):
    def __init__(self, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'success'] )
        self._set_tracking_frame = get_ros_service("/ptu_follow_frame/set_following",
                                                   StartFollowing)
        self._debug_mode = debug_mode
        self._debug_services = debug_services

    def execute(self, userdata):
        try:
            # start transformation
            self._set_tracking_frame("cluster")
            return "success"
        except:
            return "error"

class StopPTUTrack(smach.State):
    def __init__(self, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'success'] )
        self._stop_tracking_frame = get_ros_service("/ptu_follow_frame/stop_following",
                                                    Empty)
        self._debug_mode = debug_mode
        self._debug_services = debug_services

    def execute(self, userdata):
        try:
            # stop transformation
            self._stop_tracking_frame()
            return "success"
        except:
            return "error"
