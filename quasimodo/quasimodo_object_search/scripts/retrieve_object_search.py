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
from quasimodo_msgs.msg import retrieval_query
from geometry_msgs.msg import Pose
import time

UPDATE_INT_MINUTES = 100000.0

pub = ()

def retrieval_callback(object_id):

    world_model = World(server_host='localhost',server_port=62345)
    print(object_id)
    wo = world_model.get_object(object_id)

    transforms = [] #wo._poses
    images = []
    depths = []
    masks = []
    cameras = []
    req = mask_pointcloudsRequest()
    for obs, pose in zip(wo._observations, wo._poses):

        images.append(fo.get_message("/head_xtion/rgb/image_rect_color"))
        depthmat = np.zeros((480,640,1), np.uint16)
        # have to be careful here, Johan has a different encoding
        depths.append(CvBridge().cv2_to_imgmsg(depthmat, "mono16"))
        masks.append(fo.get_message("rgb_mask"))
        cameras = fo.get_message("/head_xtion/depth_registered/sw_registered/camera_info")
        req.clouds.append(obs.get_message("/head_xtion/depth_registered/points"))
        req.masks.append(obs.get_message("rgb_mask"))

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
        return

    query = retrieval_query()
    query.camera = cameras[0]
    query.cloud = resp.processed_cloud
    query.depth = depths[0]
    query.image = images[0]
    query.mask = masks[0]
    query.number_query = 10
    query.room_transform = transforms[0]

    pub.publish(query)

    print("done")

if __name__ == '__main__':
    rospy.init_node('retrieve_object_search', anonymous = False)
    pub = rospy.Publisher("/models/query", retrieval_query)
    rospy.spin()
