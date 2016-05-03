#!/usr/bin/python
import rospy
from world_state.state import World, Object
from world_state.identification import ObjectIdentification
from world_state.observation import TransformationStore, MessageStoreObject, Observation, DEFAULT_TOPICS
from world_state.geometry import Pose
from sensor_msgs.msg import Image, PointCloud2

import world_state.objectmaster as objectmaster

from mongodb_store.message_store import MessageStoreProxy

from strands_perception_msgs.msg import Table

if __name__ == '__main__':
    ''' Main Program '''
    rospy.init_node("observation_tester")
    pub = rospy.Publisher("git_image", Image, latch=1)
    
    w = World()
    om =  objectmaster.ObjectMaster()

    rospy.loginfo("Creating object.")
    ob = w.create_object()
    observation = Observation.make_observation(DEFAULT_TOPICS)
    ob.add_observation(observation)
    print ob.name
    
    tf =  TransformationStore.msg_to_transformer(observation.get_message("/tf"))


    print tf.lookupTransform("/map", "/head_xtion_rgb_optical_frame", rospy.Time(0))
    print tf.lookupTransform("/map", "/head_xtion_rgb_optical_frame",
                             observation.get_message("/head_xtion/rgb/camera_info").header.stamp)
    
    
