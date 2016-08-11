#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import PointCloud2, PointField
#from world_modeling.srv import *
from cv_bridge import CvBridge, CvBridgeError
import cv2

# SOMA2 stuff
from soma2_msgs.msg import SOMA2Object
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
from quasimodo_msgs.srv import insert_model, insert_modelRequest, insert_modelResponse
from geometry_msgs.msg import Pose
import time

def insert_model_cb():
    msg_store = MessageStoreProxy(database='world_state', collection='quasimodo')

    transforms = []
    req = mask_pointcloudsRequest()
    for cloud, mask, pose in zip(req.model.clouds, req.model.masks, req.model.local_poses):
        req.clouds.append(cloud)
        req.masks.append(mask)

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
        return False

    req = ProcessRegisteredViewsRequest()
    req.registered_views = resp.clouds
    req.registered_views_transforms = transforms

    print "Waiting for surfelize_server..."
    rospy.wait_for_service('surfelize_server')
    try:
        surfelize_server = rospy.ServiceProxy('surfelize_server', ProcessRegisteredViews)
        resp = surfelize_server(req)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return False

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
        return False

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
        return False

    new_obj.vocabulary_id = resp.id
    msg_store.update_id(object_id, new_obj)
    # Now, simply save this in mongodb, or maybe extract features first?

    print object_id

    print("done")

    resp = insert_modelResponse()
    resp.vocabulary_id = new_obj.vocabulary_id
    resp.object_id = object_id

    return resp

def remove_model_cb(req):

    msg_store = MessageStoreProxy(database='world_state', collection='quasimodo')
    # msg_store.delete(req.object_id)
    # resp = insert_modelResponse()
    # resp.object_id = req.object_id

    new_obj = msg_store.query_id(req.object_id, 'quasimodo_msgs/fused_world_state_object')[0]
    #print new_obj
    now = datetime.now()
    new_obj.removed_at = now.strftime("%Y-%m-%d %H:%M:%S")
    msg_store.update_id(req.object_id, new_obj)

    resp = insert_modelResponse()
    resp.vocabulary_id = new_obj.vocabulary_id
    resp.object_id = req.object_id

    return resp

def service_callback(req):

    if req.action == insert_modelRequest.REMOVE:
        resp = remove_model_cb(req)
        if not resp:
            print "Failed to remove model..."
    elif req.action == insert_modelRequest.INSERT:
        resp = insert_model_cb(req)
        if not resp:
            print "Failed to remove model..."
    else:
        resp = False
        print "Service options are 1: INSERT and 2: REMOVE"

    return resp

if __name__ == '__main__':
    rospy.init_node('insert_object_server', anonymous = False)

    service = rospy.Service('insert_model_service', insert_model, service_callback)

    rospy.spin()
