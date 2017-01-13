#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
#from world_modeling.srv import *
from cv_bridge import CvBridge, CvBridgeError
import cv2

# SOMA2 stuff
from soma_msgs.msg import SOMAObject
from mongodb_store.message_store import MessageStoreProxy
from soma_manager.srv import *
from geometry_msgs.msg import Pose, Transform
from observation_registration_services.srv import ProcessRegisteredViews, ProcessRegisteredViewsRequest, ProcessRegisteredViewsResponse
from soma_io.state import World, Object
from datetime import datetime, timedelta
from quasimodo_msgs.msg import fused_world_state_object
from quasimodo_msgs.srv import transform_cloud, transform_cloudRequest, transform_cloudResponse
from quasimodo_msgs.srv import index_cloud, index_cloudRequest, index_cloudResponse
from quasimodo_msgs.srv import mask_pointclouds, mask_pointcloudsRequest, mask_pointcloudsResponse
from geometry_msgs.msg import Pose
import time

UPDATE_INT_MINUTES = 10.0

def chron_callback():

    print("making query")
    soma_query = rospy.ServiceProxy('soma/query_objects',SOMAQueryObjs)
    print("done query")

    # Query all observations during the last 30 mins
    query = SOMAQueryObjsRequest()
    query.query_type = 0
    query.objecttypes=['unknown']
    query.uselowertime = True
    query.usedates = True
    dt_obj = datetime.now() - timedelta(minutes=UPDATE_INT_MINUTES) #fromtimestamp(query.lowerdate
    query.lowerhour = dt_obj.hour
    print "Hour %d" % query.lowerhour
    query.lowerminutes = dt_obj.minute
    print "Minute %d" % query.lowerminutes

    response = soma_query(query)

    if not response.objects:
        print("No SOMA objects!")
    else:
        print("got " + str(len(response.objects)) + " SOMA objects")
        msg_store = MessageStoreProxy(database='world_state', collection='quasimodo')
        # This hardcoding is no good!
        world_model = World(server_host='localhost',server_port=62345)
        #print world_model
        for x in response.objects:
            print(x.id)
            wo = world_model.get_object(x.id)

            transforms = [] #wo._poses
            req = mask_pointcloudsRequest()
            for obs, pose in zip(wo._observations, wo._poses):

                #rgb = fo.get_message("/head_xtion/rgb/image_rect_color")
                req.clouds.append(obs.get_message("/head_xtion/depth_registered/points"))
                req.masks.append(obs.get_message("rgb_mask"))

                #pointclouds.append(obs.get_message("object_cloud_camframe"))
                #could we set everything that's in the background to inf/0?
                transform = Transform()
                transform.rotation.x = pose.quaternion.x
                transform.rotation.y = pose.quaternion.y
                transform.rotation.z = pose.quaternion.z
                transform.rotation.w = pose.quaternion.w
                transform.translation.x = pose.position.x
                transform.translation.y = pose.position.y
                transform.translation.z = pose.position.z

                transforms.append(transform)

            print "Waiting for retrieval_segmentation_service..."
            rospy.wait_for_service('retrieval_segmentation_service')
            try:
                surfelize_server = rospy.ServiceProxy('retrieval_segmentation_service', mask_pointclouds)
                resp = surfelize_server(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return

            #sensor_msgs/PointCloud2[] registered_views
            #geometry_msgs/Transform[] registered_views_transforms
            #std_msgs/Float32[]	  intrinsics # can be empty
            #---
            #sensor_msgs/PointCloud2 processed_cloud
            req = ProcessRegisteredViewsRequest()
            req.registered_views = resp.clouds
            # This should possibly be transformed to the frame of the first
            # observation to make visualization a bit easier
            req.registered_views_transforms = transforms

            print "Waiting for surfelize_server..."
            rospy.wait_for_service('surfelize_server')
            try:
                surfelize_server = rospy.ServiceProxy('surfelize_server', ProcessRegisteredViews)
                resp = surfelize_server(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return

            new_obj = fused_world_state_object()
            new_obj.surfel_cloud = resp.processed_cloud
            new_obj.object_id = x.id
            now = datetime.now()
            new_obj.inserted_at = now.strftime("%Y-%m-%d %H:%M:%S")

            for obs, pose in zip(wo._observations, wo._poses):
                new_obj.transforms.append(Pose())
                new_obj.transforms[-1].position.x = pose.position.x
                new_obj.transforms[-1].position.y = pose.position.y
                new_obj.transforms[-1].position.z = pose.position.z
                new_obj.transforms[-1].orientation.x = pose.quaternion.x
                new_obj.transforms[-1].orientation.y = pose.quaternion.y
                new_obj.transforms[-1].orientation.z = pose.quaternion.z
                new_obj.transforms[-1].orientation.w = pose.quaternion.w

            print "Waiting for retrieval_features_service..."
            rospy.wait_for_service('retrieval_features_service')
            req = transform_cloudRequest()
            req.cloud = resp.processed_cloud
            try:
                surfelize_server = rospy.ServiceProxy('retrieval_features_service', transform_cloud)
                resp = surfelize_server(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return

            new_obj.features = resp.cloud1
            new_obj.keypoints = resp.cloud2

            object_id = msg_store.insert(new_obj)

            print "Waiting for retrieval_vocabulary_service..."
            rospy.wait_for_service('retrieval_vocabulary_service')
            req = index_cloudRequest()
            req.cloud = new_obj.features
            req.object_id = object_id
            try:
                surfelize_server = rospy.ServiceProxy('retrieval_vocabulary_service', index_cloud)
                resp = surfelize_server(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                return

            new_obj.vocabulary_id = resp.id
            msg_store.update_id(object_id, new_obj)
            # Now, simply save this in mongodb, or maybe extract features first?

            print object_id

            #break # if you only want to add one object



    print("done")


if __name__ == '__main__':
    rospy.init_node('create_object_search_observation', anonymous = False)
    print("getting soma service")
    rospy.wait_for_service('soma/query_db')
    print("done initializing")

    r = rospy.Rate(1.0/(60.0*UPDATE_INT_MINUTES)) # 10hz
    while not rospy.is_shutdown():
        chron_callback()
        break
        r.sleep()
