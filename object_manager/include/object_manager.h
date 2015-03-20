#ifndef __SEMANTIC_MAP_PUBLISHER__H
#define __SEMANTIC_MAP_PUBLISHER__H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>


// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

#include <mongodb_store/message_store.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

// Services
#include <semantic_map_publisher/MetaroomService.h>
#include <object_manager/DynamicObjectsService.h>
#include <semantic_map_publisher/ObservationService.h>
#include <semantic_map_publisher/ObservationOctomapService.h>
#include <semantic_map_publisher/WaypointInfoService.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <pcl/registration/distances.h>
#include <pcl/search/kdtree.h>
#include "load_utilities.h"

#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/filters/voxel_grid.h>


template <class PointType>
class ObjectManager {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    typedef typename object_manager::DynamicObjectsService::Request DynamicObjectsServiceRequest;
    typedef typename object_manager::DynamicObjectsService::Response DynamicObjectsServiceResponse;

    struct ObjStruct {
        std::string id;
        CloudPtr cloud;
    };




    ObjectManager(ros::NodeHandle nh);
    ~ObjectManager();

    bool dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res);
    std::vector<ObjectManager<PointType>::ObjStruct>  loadDynamicObjectsFromObservation(std::string obs_file);

    ros::Publisher                                                              m_PublisherDynamicClusters;
    ros::ServiceServer                                                          m_DynamicObjectsServiceServer;

    static CloudPtr filterGroundClusters(CloudPtr dynamic, double min_height)
    {
        CloudPtr filtered(new Cloud());


        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud (dynamic);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-1.0, min_height);
        pass.setFilterLimitsNegative (true);
        pass.filter (*filtered);

        return filtered;
    }


private:
    ros::NodeHandle                                                             m_NodeHandle;
    std::string                                                                 m_dataFolder;
    std::map<std::string, std::vector<ObjStruct>>                               m_waypointToObjMap;
    std::map<std::string, std::string>                                          m_waypointToSweepFileMap;
};

template <class PointType>
ObjectManager<PointType>::ObjectManager(ros::NodeHandle nh)
{
    ROS_INFO_STREAM("ObjectManager node initialized");

    m_NodeHandle = nh;

    passwd* pw = getpwuid(getuid());
    std::string default_folder(pw->pw_dir);
    default_folder+="/.semanticMap/";

    m_NodeHandle.param<std::string>("object_folder",m_dataFolder,default_folder);
    ROS_INFO_STREAM("Reading dynamic object from "<<m_dataFolder);

    m_PublisherDynamicClusters = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/object_manager/objects", 1, true);

    m_DynamicObjectsServiceServer = m_NodeHandle.advertiseService("ObjectManager/DynamicObjectsService", &ObjectManager::dynamicObjectsServiceCallback, this);
}

template <class PointType>
ObjectManager<PointType>::~ObjectManager()
{

}

template <class PointType>
bool ObjectManager<PointType>::dynamicObjectsServiceCallback(DynamicObjectsServiceRequest &req, DynamicObjectsServiceResponse &res)
{
    ROS_INFO_STREAM("Received a dynamic clusters request for waypoint "<<req.waypoint_id);

    using namespace std;
    std::vector<std::string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(m_dataFolder, req.waypoint_id);
    if (matchingObservations.size() == 0)
    {
        ROS_INFO_STREAM("No observations for this waypoint "<<req.waypoint_id);
        return false;
    }

    sort(matchingObservations.begin(), matchingObservations.end());
    reverse(matchingObservations.begin(), matchingObservations.end());
    string latest = matchingObservations[0];
    std::vector<ObjStruct> currentObjects;

    ROS_INFO_STREAM("Latest observation "<<latest);

    auto it = m_waypointToSweepFileMap.find(req.waypoint_id);
    if (it == m_waypointToSweepFileMap.end())
    {
        // no dynamic clusters loaded for this waypoint
        // -> load them
        std::vector<ObjStruct> dynamicObjects = loadDynamicObjectsFromObservation(latest);
        if (dynamicObjects.size() == 0)
        {
            ROS_INFO_STREAM("No objects detected after clustering.");
            return true;
        }

        m_waypointToSweepFileMap[req.waypoint_id] = latest;
        m_waypointToObjMap[req.waypoint_id] = dynamicObjects;

        currentObjects = m_waypointToObjMap[req.waypoint_id];
    } else {
        if (m_waypointToSweepFileMap[req.waypoint_id] != latest)
        {
            ROS_INFO_STREAM("Older point cloud loaded in memory. Loading objects from latest observation ...");
            std::vector<ObjStruct> dynamicObjects = loadDynamicObjectsFromObservation(latest);
            if (dynamicObjects.size() == 0)
            {
                ROS_INFO_STREAM("No objects detected after clustering.");
                return true;
            }

            m_waypointToSweepFileMap[req.waypoint_id] = latest;
            m_waypointToObjMap[req.waypoint_id] = dynamicObjects;

            currentObjects = m_waypointToObjMap[req.waypoint_id];
        } else {
            ROS_INFO_STREAM("Objects loaded in memory");
            auto it2 =  m_waypointToObjMap.find(req.waypoint_id);
            if (it2 == m_waypointToObjMap.end())
            {
                ROS_ERROR_STREAM("Object map is empty. Reload.");
                std::vector<ObjStruct> dynamicObjects = loadDynamicObjectsFromObservation(latest);
                if (dynamicObjects.size() == 0)
                {
                    ROS_INFO_STREAM("No objects detected after clustering.");
                    return true;
                }

                m_waypointToSweepFileMap[req.waypoint_id] = latest;
                m_waypointToObjMap[req.waypoint_id] = dynamicObjects;

                currentObjects = m_waypointToObjMap[req.waypoint_id];
            } else {
                currentObjects = m_waypointToObjMap[req.waypoint_id];
            }
        }
    }

    ROS_INFO_STREAM("Found "<<currentObjects.size() <<" objects at "<<req.waypoint_id);
    // publish objects
    CloudPtr allObjects(new Cloud());

    for (auto cloudObj : currentObjects)
    {
        *allObjects += *(cloudObj.cloud);
    }

    sensor_msgs::PointCloud2 msg_objects;
    pcl::toROSMsg(*allObjects, msg_objects);
    msg_objects.header.frame_id="/map";

    m_PublisherDynamicClusters.publish(msg_objects);

    std::vector<sensor_msgs::PointCloud2> clouds;
    std::vector<std::string> id;
    for (auto cloudObj : currentObjects)
    {
        sensor_msgs::PointCloud2 msg_objects;
        pcl::toROSMsg(*cloudObj.cloud, msg_objects);
        msg_objects.header.frame_id="/map";

        clouds.push_back(msg_objects);
        id.push_back(cloudObj.id);
    }

    res.objects = clouds;
    res.object_id = id;

    return true;


}

template <class PointType>
std::vector<typename ObjectManager<PointType>::ObjStruct>  ObjectManager<PointType>::loadDynamicObjectsFromObservation(std::string obs_file)
{
    std::vector<ObjStruct> dynamicObjects;
    auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(obs_file, std::vector<std::string>{"RoomDynamicClusters"},false);
    if (sweep.dynamicClusterCloud->points.size() == 0)
    {
        // no clusters in the observation
        return dynamicObjects;
    }

    sweep.dynamicClusterCloud = filterGroundClusters(sweep.dynamicClusterCloud, 0.2);

    double tolerance = 0.03;
    int min_cluster_size = 800;
    int max_cluster_size = 10000;
    // split into clusters
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (sweep.dynamicClusterCloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (sweep.dynamicClusterCloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        CloudPtr cloud_cluster (new Cloud());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (sweep.dynamicClusterCloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::stringstream ss;ss<<"object_";ss<<j;
        ObjStruct object;
        object.cloud = cloud_cluster;
        object.id = ss.str();

        dynamicObjects.push_back(object);

        j++;
    }

    return dynamicObjects;
}


#endif
