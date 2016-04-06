import rospy
import smach

from camera_srv_definitions.srv import start_tracker, stop_tracker, visualize_compound,get_tracking_results
from incremental_object_learning_srv_definitions.srv import learn_object, save_model
from recognition_srv_definitions.srv import get_configuration, recognize
from geometry_msgs.msg import Transform

from std_msgs.msg import String, Int32MultiArray
from util import get_ros_service

from world_state.observation import MessageStoreObject, Observation, TransformationStore
from world_state.identification import ObjectIdentification
from world_state.state import World, Object
from object_manager.msg import DynamicObjectTracks
from geometry_msgs.msg import TransformStamped
from static_transform_manager.srv import SetTransformation, StopTransformation
import tf
import os

class StartCameraTrack(smach.State):
    def __init__(self, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'success','preempted'] )
        self._start_camera_tracker = get_ros_service("/camera_tracker/start_recording", start_tracker)       
        self._set_transform = get_ros_service("/static_transforms_manager/set_tf",
                                              SetTransformation)
        self._debug_mode = debug_mode
        self._debug_services = debug_services


    def execute(self, userdata):
        try:
            if self._debug_mode:
                self._debug_services.set_status("Turning on camera tracking.[OK]")
                proceed = self._debug_services.get_proceed(("OK",))
                if proceed == "PREEMPT":
                    return 'preempted'
            trans = TransformStamped()

            # get transform map->head_xtion_rgb_optical
            listener = tf.TransformListener()
            listener.waitForTransform("/map", "/head_xtion_rgb_optical_frame", rospy.Time(), rospy.Duration(5.0))
            try:
                (tran,rot) = listener.lookupTransform('/map', '/head_xtion_rgb_optical_frame', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("Bail because cant find intial transform to the fucking camera.")
                return "error"

            print "tran=",tran
            print "rot=",rot
            t=trans.transform.translation
            t.x, t.y,t.z = tran
            r=trans.transform.rotation
            r.x,r.y,r.z,r.w=rot

            trans.header.frame_id = "map"
            trans.child_frame_id = "world"
            rospy.loginfo("SETTING TRANSFORM")
            self._set_transform(trans)

            # start transformation
            rospy.loginfo("STARTING CAMERA TACKER")
            self._start_camera_tracker()
            # need to wait for camera trackers initial frame drop to finish
            rospy.loginfo("Giving camera tracker 10s to init...")
            rospy.sleep(10)
            return "success"
        except Exception, e:
            rospy.loginfo("ERROR:"+str(e))
            return "error"

class StopCameraTrack(smach.State):
    def __init__(self, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'success'] )
        self._stop_camera_tracker = get_ros_service("/camera_tracker/stop_recording", stop_tracker)
        self._stop_transform = get_ros_service("/static_transforms_manager/stop_tf",
                                               StopTransformation)

    def execute(self, userdata):
        try:
            # stop transformation
            self._stop_camera_tracker()
            self._stop_transform("world")
            return "success"
        except:
            return "error"

class LearnObjectModel(smach.StateMachine):
    def __init__(self, model_path, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'done','preempted'],
                              input_keys=['dynamic_object','object'],
                              output_keys=['object'])
        
        self._model_path = model_path
        self._get_tracking_results = get_ros_service("/camera_tracker/get_results", get_tracking_results)
        ## Object learning
        self._learn_object_model = get_ros_service("/incremental_object_learning/learn_object", learn_object)
        self._save_object_model = get_ros_service("/incremental_object_learning/save_model", save_model)

        self._tracks_publisher = rospy.Publisher("/object_learning/dynamic_object_tracks", DynamicObjectTracks)
        self._status_publisher =  rospy.Publisher("/object_learning/status",
                                                  String)
        self._debug_mode = debug_mode
        self._debug_services=debug_services
        ## Recognition services
        #self._get_rec_configuration = get_ros_service(
        #        "/recognition_service/get_configuration", get_configuration)
        #recognize_object = get_ros_service(
        #        "/recognition_service/mp_recognition", recognize)



    def execute(self, userdata):
        try:
            if self._debug_mode:
                self._debug_services.set_status("Sending frames to learn. [OK]")
                proceed = self._debug_services.get_proceed(("OK",))
                if proceed == "PREEMPT":
                    return 'preempted'

            tracks = self._get_tracking_results()
            rospy.loginfo("Got some tracking results!")
            rospy.loginfo("Number of keyframes:%d"%len(tracks.keyframes))
            rospy.loginfo("Transforms:")
            transforms=[Transform()]
            transforms[0].rotation.w=1 # this was z ! why was it z?
            transforms.extend(tracks.transforms)
            frames = [userdata.dynamic_object.object_cloud]
            frames.extend(tracks.keyframes)
            rospy.loginfo(tracks.transforms)
            print "About to call object learning"
            #print userdata.dynamic_object.object_mask
            for f in frames:
                print "Frame width x height was: %d x %d. Changing it."%(f.width, f.height)
                f.width = 640
                f.height = 480

            # Save the camera track to the DB, and to rombusdb
            tracks_msg = DynamicObjectTracks()
            tracks_msg.poses = transforms
            tracks_msg.clouds = frames
            self._tracks_publisher.publish(tracks_msg)
            rospy.sleep(1)
            self._status_publisher.publish(String("stop_viewing"))
            # split it for message store size limit
            for img, trans in zip(frames,transforms):
                track_msg = DynamicObjectTracks()
                track_msg.poses=[trans]
                track_msg.clouds=[img]
                stored = MessageStoreObject.create(track_msg)
                userdata['object'].add_msg_store(stored)

            # Get the mask and store it, stupid fool
            mask = Int32MultiArray()
            mask.data = userdata.dynamic_object.object_mask
            stored = MessageStoreObject.create(mask)
            userdata['object'].add_msg_store(stored)
            print "Stored the mask"

            self._learn_object_model(frames, transforms,
                                     userdata.dynamic_object.object_mask)
            rospy.loginfo("Saving learnt model..")
            self._save_object_model(String(userdata['object'].name),
                                    String(self._model_path))
            return 'done'
        except Exception, e:
            print e
            return "error"
