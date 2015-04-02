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
#include "load_utilities.h"

#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl/filters/voxel_grid.h>

template <class PointType>
class SemanticMapPublisher {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    typedef typename semantic_map_publisher::MetaroomService::Request MetaroomServiceRequest;
    typedef typename semantic_map_publisher::MetaroomService::Response MetaroomServiceResponse;
    typedef typename semantic_map_publisher::ObservationService::Request ObservationServiceRequest;
    typedef typename semantic_map_publisher::ObservationService::Response ObservationServiceResponse;
//    typedef typename semantic_map_publisher::DynamicClusterService::Request DynamicClusterServiceRequest;
//    typedef typename semantic_map_publisher::DynamicClusterService::Response DynamicClusterServiceResponse;
    typedef typename semantic_map_publisher::WaypointInfoService::Request WaypointInfoServiceRequest;
    typedef typename semantic_map_publisher::WaypointInfoService::Response WaypointInfoServiceResponse;
    typedef typename semantic_map_publisher::ObservationOctomapService::Request ObservationOctomapServiceRequest;
    typedef typename semantic_map_publisher::ObservationOctomapService::Response ObservationOctomapServiceResponse;

    struct ObsStruct {
        std::string file;
        CloudPtr completeCloud;
    };




    SemanticMapPublisher(ros::NodeHandle nh);
    ~SemanticMapPublisher();

    bool metaroomServiceCallback(MetaroomServiceRequest &req, MetaroomServiceResponse &res);
//    bool dynamicClusterServiceCallback(DynamicClusterServiceRequest &req, DynamicClusterServiceResponse &res);
    bool observationServiceCallback(ObservationServiceRequest &req, ObservationServiceResponse &res);
    bool observationOctomapServiceCallback(ObservationOctomapServiceRequest &req, ObservationOctomapServiceResponse &res);
    bool waypointInfoServiceCallback(WaypointInfoServiceRequest &req, WaypointInfoServiceResponse &res);

    ros::Publisher                                                              m_PublisherMetaroom;
    ros::Publisher                                                              m_PublisherObservation;
    ros::Publisher                                                              m_PublisherDynamicClusters;
    ros::ServiceServer                                                          m_MetaroomServiceServer;
//    ros::ServiceServer                                                          m_DynamicClusterServiceServer;
    ros::ServiceServer                                                          m_ObservationServiceServer;
    ros::ServiceServer                                                          m_ObservationOctomapServiceServer;
    ros::ServiceServer                                                          m_WaypointInfoServiceServer;


private:
    ros::NodeHandle                                                             m_NodeHandle;
    std::string                                                                 m_dataFolder;
    std::map<std::string, ObsStruct>                                            m_waypointToObsMap;
    std::map<std::string, ObsStruct>                                            m_waypointToDynClMap;
};

template <class PointType>
SemanticMapPublisher<PointType>::SemanticMapPublisher(ros::NodeHandle nh)
{
    ROS_INFO_STREAM("Semantic map publisher node initialized");

    m_NodeHandle = nh;

    passwd* pw = getpwuid(getuid());
    std::string default_folder(pw->pw_dir);
    default_folder+="/.semanticMap/";

    m_NodeHandle.param<std::string>("semantic_map_folder",m_dataFolder,default_folder);
    ROS_INFO_STREAM("Publishing semantic map data from "<<m_dataFolder);

    m_PublisherMetaroom = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/metaroom", 1, true);
    m_PublisherDynamicClusters = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/dynamic_clusters", 1, true);
    m_PublisherObservation = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/observation", 1, true);

    m_MetaroomServiceServer = m_NodeHandle.advertiseService("SemanticMapPublisher/MetaroomService", &SemanticMapPublisher::metaroomServiceCallback, this);
//    m_DynamicClusterServiceServer = m_NodeHandle.advertiseService("SemanticMapPublisher/DynamicClusterService", &SemanticMapPublisher::dynamicClusterServiceCallback, this);
    m_ObservationServiceServer = m_NodeHandle.advertiseService("SemanticMapPublisher/ObservationService", &SemanticMapPublisher::observationServiceCallback, this);
    m_ObservationOctomapServiceServer = m_NodeHandle.advertiseService("SemanticMapPublisher/ObservationOctomapService", &SemanticMapPublisher::observationOctomapServiceCallback, this);
    m_WaypointInfoServiceServer= m_NodeHandle.advertiseService("SemanticMapPublisher/WaypointInfoService", &SemanticMapPublisher::waypointInfoServiceCallback, this);
}

template <class PointType>
SemanticMapPublisher<PointType>::~SemanticMapPublisher()
{

}


template <class PointType>
bool SemanticMapPublisher<PointType>::metaroomServiceCallback(MetaroomServiceRequest &req, MetaroomServiceResponse &res)
{
    ROS_INFO_STREAM("Received a metaroom request for waypoint "<<req.waypoint_id<<". Not supported for now.");

}

template <class PointType>
bool SemanticMapPublisher<PointType>::observationServiceCallback(ObservationServiceRequest &req, ObservationServiceResponse &res)
{
    using namespace std;
    ROS_INFO_STREAM("Received an observation request for waypoint "<<req.waypoint_id);

    std::vector<std::string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(m_dataFolder, req.waypoint_id);
    if (matchingObservations.size() == 0)
    {
        ROS_INFO_STREAM("No observations for this waypoint "<<req.waypoint_id);
        return true;
    }
    sort(matchingObservations.begin(), matchingObservations.end());
    reverse(matchingObservations.begin(), matchingObservations.end());
    string latest = matchingObservations[0];

    auto it = m_waypointToObsMap.find(req.waypoint_id);

    CloudPtr toPublish(new Cloud());
    if (it == m_waypointToObsMap.end())
    {
        // no observation loaded and stored in memory for this waypoint
        // -> load it
        ROS_INFO_STREAM("Point cloud not loaded in memory. Loading ...");
        CloudPtr completeCloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointType>(latest);
        ObsStruct latestObs;latestObs.completeCloud = completeCloud; latestObs.file = latest;
        m_waypointToObsMap[req.waypoint_id] = latestObs;
        *toPublish = *completeCloud;
    } else {
        if (m_waypointToObsMap[req.waypoint_id].file != latest)
        {
            ROS_INFO_STREAM("Older point cloud loaded in memory. Loading latest observation ...");
            CloudPtr completeCloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointType>(latest);
            ObsStruct latestObs;latestObs.completeCloud = completeCloud; latestObs.file = latest;
            m_waypointToObsMap[req.waypoint_id] = latestObs;
            *toPublish = *completeCloud;
        } else {
            *toPublish = *m_waypointToObsMap[req.waypoint_id].completeCloud;
        }
    }

    CloudPtr observationCloud(new Cloud());
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud (toPublish);
    vg.setLeafSize (req.resolution, req.resolution, req.resolution);
    vg.filter (*observationCloud);

//    CloudPtr observationCloud = toPublish;
    sensor_msgs::PointCloud2 msg_observation;
    pcl::toROSMsg(*observationCloud, msg_observation);
    msg_observation.header.frame_id="/map";

    res.cloud = msg_observation;

    // also publish on topic
    m_PublisherObservation.publish(msg_observation);
    return true;

}

template <class PointType>
bool SemanticMapPublisher<PointType>::waypointInfoServiceCallback(WaypointInfoServiceRequest &req, WaypointInfoServiceResponse &res)
{
    SimpleSummaryParser summary_parser;
    summary_parser.createSummaryXML(m_dataFolder);
    auto sweep_xmls = summary_parser.getRooms();

    // first construct waypoint id to sweep xml map
    std::map<std::string, std::vector<std::string>> waypointToSweepsMap;
    for (size_t i=0; i<sweep_xmls.size(); i++)
    {
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), false);
        waypointToSweepsMap[sweep.roomWaypointId].push_back(sweep_xmls[i].roomXmlFile);
    }

    for (auto it = waypointToSweepsMap.begin(); it!= waypointToSweepsMap.end(); it++)
    {
        res.waypoint_id.push_back(it->first);
        res.observation_count.push_back(it->second.size());
    }
    return true;
}

template <class PointType>
bool SemanticMapPublisher<PointType>::observationOctomapServiceCallback(ObservationOctomapServiceRequest &req, ObservationOctomapServiceResponse &res)
{
    ROS_INFO_STREAM("Received an observation octomap request for waypoint "<<req.waypoint_id);
    using namespace std;
    std::vector<std::string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(m_dataFolder, req.waypoint_id);
    if (matchingObservations.size() == 0)
    {
        ROS_INFO_STREAM("No observations for this waypoint "<<req.waypoint_id);
        return true;
    }

    sort(matchingObservations.begin(), matchingObservations.end());
    reverse(matchingObservations.begin(), matchingObservations.end());
    string latest = matchingObservations[0];

    auto it = m_waypointToObsMap.find(req.waypoint_id);

    CloudPtr toPublish(new Cloud());
    if (it == m_waypointToObsMap.end())
    {
        // no observation loaded and stored in memory for this waypoint
        // -> load it
        ROS_INFO_STREAM("Point cloud not loaded in memory. Loading ...");
        CloudPtr completeCloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointType>(latest);
        ObsStruct latestObs;latestObs.completeCloud = completeCloud; latestObs.file = latest;
        m_waypointToObsMap[req.waypoint_id] = latestObs;
        *toPublish = *completeCloud;
    } else {
        if (m_waypointToObsMap[req.waypoint_id].file != latest)
        {
            ROS_INFO_STREAM("Older point cloud loaded in memory. Loading latest observation ...");
            CloudPtr completeCloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointType>(latest);
            ObsStruct latestObs;latestObs.completeCloud = completeCloud; latestObs.file = latest;
            m_waypointToObsMap[req.waypoint_id] = latestObs;
            *toPublish = *completeCloud;
        } else {
            *toPublish = *m_waypointToObsMap[req.waypoint_id].completeCloud;
        }
    }
    CloudPtr observationCloud = toPublish;
    sensor_msgs::PointCloud2 msg_observation;
    pcl::toROSMsg(*toPublish, msg_observation);
    msg_observation.header.frame_id="/map";

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*toPublish, centroid);
    octomap::point3d octo_centroid(centroid[0],centroid[1],centroid[2]);
    // convert observation cloud to octomap and return it
    octomap::Pointcloud oct_pc;
    octomap::pointCloud2ToOctomap(msg_observation, oct_pc);
    octomap::OcTree map(req.resolution);
    map.insertPointCloud(oct_pc, octo_centroid);
    octomap_msgs::Octomap octo_msg;
    octomap_msgs::fullMapToMsg(map,octo_msg);
    res.octomap = octo_msg;
    return true;
}

//template <class PointType>
//bool SemanticMapPublisher<PointType>::dynamicClusterServiceCallback(DynamicClusterServiceRequest &req, DynamicClusterServiceResponse &res)
//{
//    ROS_INFO_STREAM("Received a dynamic clusters request for waypoint "<<req.waypoint_id<<". Not supported for now.");
//}


#endif
