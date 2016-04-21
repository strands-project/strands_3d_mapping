import rospy
import smach

# from camera_srv_definitions.srv import start_tracker, stop_tracker, visualize_compound,get_tracking_results
from observation_registration_services.srv import GetLastAdditionalViewRegistrationResultService
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

class LearnObjectModelYear3(smach.StateMachine):
    def __init__(self, model_path, debug_mode, debug_services):
        smach.State.__init__( self, outcomes=['error', 'done','preempted'],
                              input_keys=['dynamic_object','object'],
                              output_keys=['object'])

        self._model_path = model_path
        # self._get_tracking_results = get_ros_service("/camera_tracker/get_results", get_tracking_results)
        self._get_additional_view_registration_results = get_ros_service("get_last_additional_view_registration_results_server", GetLastAdditionalViewRegistrationResultService)
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

            # Wait for the processing of the object


            # tracks = self._get_tracking_results()
            registration_data = self.get_last_additional_view_registration_results()
            rospy.loginfo("Got some registration results!")
            rospy.loginfo("Number of keyframes:%d"%len(registration_data.additional_views))
            rospy.loginfo("Number of registered transforms:%d"%len(registration_data.additional_view_transforms))
            rospy.loginfo("Transforms:")
            # transforms=[Transform()]
            # transforms[0].rotation.w=1 # this was z ! why was it z?
            transforms=[userdata.dynamic_object.transform_to_map * registration_data.observation_transform.inverse()]
            transforms.extend(registration_data.additional_view_transforms)
            frames = [userdata.dynamic_object.object_cloud]
            frames.extend(registration_data.additional_views)
            rospy.loginfo(registration_data.additional_views)
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
            split it for message store size limit
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
