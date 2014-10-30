#ifndef __SEMANTIC_MAP_NODE__H
#define __SEMANTIC_MAP_NODE__H

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

#include <mongodb_store/message_store.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

// Services
#include <semantic_map/MetaroomService.h>
#include <semantic_map/DynamicClusterService.h>
#include <semantic_map/ObservationService.h>

#include <semantic_map/RoomObservation.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "roomXMLparser.h"
#include "metaroomXMLparser.h"
#include "semanticMapSummaryParser.h"

template <class PointType>
class SemanticMapNode {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename SemanticMapSummaryParser<PointType>::EntityStruct Entities;

    typedef typename semantic_map::MetaroomService::Request MetaroomServiceRequest;
    typedef typename semantic_map::MetaroomService::Response MetaroomServiceResponse;
    typedef typename semantic_map::MetaroomService::Request ObservationServiceRequest;
    typedef typename semantic_map::MetaroomService::Response ObservationServiceResponse;
    typedef typename semantic_map::DynamicClusterService::Request DynamicClusterServiceRequest;
    typedef typename semantic_map::DynamicClusterService::Response DynamicClusterServiceResponse;


    typedef typename std::map<std::string, boost::shared_ptr<MetaRoom<PointType> > >::iterator WaypointMetaroomMapIterator;
    typedef typename std::map<std::string, SemanticRoom<PointType> >::iterator WaypointRoomMapIterator;
    typedef typename std::map<std::string, CloudPtr>::iterator WaypointPointCloudMapIterator;

    SemanticMapNode(ros::NodeHandle nh);
    ~SemanticMapNode();

    void processRoomObservation(std::string xml_file_name);

    void roomObservationCallback(const semantic_map::RoomObservationConstPtr& obs_msg);
    bool metaroomServiceCallback(MetaroomServiceRequest &req, MetaroomServiceResponse &res);
    bool dynamicClusterServiceCallback(DynamicClusterServiceRequest &req, DynamicClusterServiceResponse &res);
    bool observationServiceCallback(ObservationServiceRequest &req, ObservationServiceResponse &res);

    ros::Subscriber                                                             m_SubscriberRoomObservation;
    ros::Publisher                                                              m_PublisherMetaroom;
    ros::Publisher                                                              m_PublisherObservation;
    ros::Publisher                                                              m_PublisherDynamicClusters;
    ros::ServiceServer                                                          m_MetaroomServiceServer;
    ros::ServiceServer                                                          m_DynamicClusterServiceServer;
    ros::ServiceServer                                                          m_ObservationServiceServer;

private:
    ros::NodeHandle                                                             m_NodeHandle;
    SemanticMapSummaryParser<PointType>                                         m_SummaryParser;
    bool                                                                        m_bSaveIntermediateData;
    std::vector<boost::shared_ptr<MetaRoom<PointType> > >                       m_vLoadedMetarooms;
    mongodb_store::MessageStoreProxy                                           m_messageStore;
    bool                                                                        m_bLogToDB;
    std::map<std::string, boost::shared_ptr<MetaRoom<PointType> > >             m_WaypointToMetaroomMap;
    std::map<std::string, SemanticRoom<PointType> >                             m_WaypointToRoomMap;
    std::map<std::string, CloudPtr>                                             m_WaypointToDynamicClusterMap;
    bool                                                                        m_bUpdateMetaroom;
};

template <class PointType>
SemanticMapNode<PointType>::SemanticMapNode(ros::NodeHandle nh) : m_messageStore(nh)
{
    ROS_INFO_STREAM("Semantic map node initialized");

    m_NodeHandle = nh;

    m_SubscriberRoomObservation = m_NodeHandle.subscribe("/local_metric_map/room_observations",1, &SemanticMapNode::roomObservationCallback,this);

    m_PublisherMetaroom = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/metaroom", 1, true);
    m_PublisherDynamicClusters = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/dynamic_clusters", 1, true);
    m_PublisherObservation = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/merged_point_cloud_downsampled", 1, true);


    m_MetaroomServiceServer = m_NodeHandle.advertiseService("SemanticMap/MetaroomService", &SemanticMapNode::metaroomServiceCallback, this);
    m_DynamicClusterServiceServer = m_NodeHandle.advertiseService("SemanticMap/DynamicClusterService", &SemanticMapNode::dynamicClusterServiceCallback, this);
    m_ObservationServiceServer = m_NodeHandle.advertiseService("SemanticMap/ObservationService", &SemanticMapNode::observationServiceCallback, this);

    bool save_intermediate;
    m_NodeHandle.param<bool>("save_intermediate",save_intermediate,false);
    if (!save_intermediate)
    {
        ROS_INFO_STREAM("NOT saving intermediate data.");
    } else {
        ROS_INFO_STREAM("Saving intermediate data.");
    }

    m_NodeHandle.param<bool>("log_to_db",m_bLogToDB,false);
    if (m_bLogToDB)
    {
        ROS_INFO_STREAM("Logging dynamic clusters to the database.");
    } else {
        ROS_INFO_STREAM("NOT logging dynamic clusters to the database.");
    }

    m_NodeHandle.param<bool>("update_metaroom",m_bUpdateMetaroom,true);
    if (m_bUpdateMetaroom)
    {
        ROS_INFO_STREAM("The metarooms will be updated with new room observations.");
    } else {
        ROS_INFO_STREAM("The metarooms will NOT be updated with new room observations.");
    }

}

template <class PointType>
SemanticMapNode<PointType>::~SemanticMapNode()
{

}


template <class PointType>
void SemanticMapNode<PointType>::processRoomObservation(std::string xml_file_name)
{
    std::cout<<"File name "<<xml_file_name<<std::endl;

    SemanticRoomXMLParser<PointType> parser;
    SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(xml_file_name,false);
    aRoom.resetRoomTransform();

    // update summary xml
    m_SummaryParser.createSummaryXML();

    // update list of rooms & metarooms
    m_SummaryParser.refresh();

    ROS_INFO_STREAM("Summary XML created.");



    boost::shared_ptr<MetaRoom<PointType> > metaroom;
    bool found = false;

    // first check if the matching metaroom has already been loaded
    for (size_t i=0; i<m_vLoadedMetarooms.size(); i++)
    {
        double centroidDistance = pcl::distances::l2(m_vLoadedMetarooms[i]->getCentroid(),aRoom.getCentroid());
        if (! (centroidDistance < ROOM_CENTROID_DISTANCE) )
        {
            continue;
        } else {
            ROS_INFO_STREAM("Matching metaroom already loaded.");
            metaroom = m_vLoadedMetarooms[i];
            found = true;
            break;
        }
    }

    // if not loaded already, look through already saved metarooms
    if (!found)
    {

        std::string matchingMetaroomXML = "";
        std::vector<Entities> allMetarooms = m_SummaryParser.getMetaRooms();
        for (size_t i=0; i<allMetarooms.size();i++)
        {
            double centroidDistance = pcl::distances::l2(allMetarooms[i].centroid,aRoom.getCentroid());
            if (! (centroidDistance < ROOM_CENTROID_DISTANCE) )
            {
                continue;
            } else {
                matchingMetaroomXML = allMetarooms[i].roomXmlFile;
                break;
            }
        }

        if (matchingMetaroomXML == "")
        {
            ROS_INFO_STREAM("No matching metaroom found. Create new metaroom.");
            metaroom =  boost::shared_ptr<MetaRoom<PointType> >(new MetaRoom<PointType>());
        } else {
            ROS_INFO_STREAM("Matching metaroom found. XML file: "<<matchingMetaroomXML);
            metaroom =  boost::shared_ptr<MetaRoom<PointType> >(new MetaRoom<PointType>(MetaRoomXMLParser<PointType>::loadMetaRoomFromXML(matchingMetaroomXML,false)));
        }
    }

    metaroom->setUpdateMetaroom(m_bUpdateMetaroom);
    metaroom->setSaveIntermediateSteps(m_bSaveIntermediateData);

    // update metaroom
    metaroom->updateMetaRoom(aRoom);

    // save metaroom
    ROS_INFO_STREAM("Saving metaroom.");
    MetaRoomXMLParser<PointType> meta_parser;
    meta_parser.saveMetaRoomAsXML(*metaroom);

    CloudPtr roomCloud = aRoom.getInteriorRoomCloud();
    CloudPtr metaroomCloud = metaroom->getInteriorRoomCloud();

    // publish data
    sensor_msgs::PointCloud2 msg_metaroom;
    pcl::toROSMsg(*metaroomCloud, msg_metaroom);
    msg_metaroom.header.frame_id="/map";
    m_PublisherMetaroom.publish(msg_metaroom);
    m_vLoadedMetarooms.push_back(metaroom);
    ROS_INFO_STREAM("Published metaroom");


    if (aRoom.getRoomStringId() != "") // waypoint id set, update the map
    {
        m_WaypointToMetaroomMap[aRoom.getRoomStringId()] = metaroom;
        ROS_INFO_STREAM("Updated map with new metaroom for waypoint "<<aRoom.getRoomStringId());

//        boost::shared_ptr<SemanticRoom<PointType> > roomPtr(&aRoom);
        m_WaypointToRoomMap[aRoom.getRoomStringId()] = aRoom;
    }


    // compute differences
    ROS_INFO_STREAM("Computing differences");
    CloudPtr difference(new Cloud());

    pcl::SegmentDifferences<PointType> segment;
    segment.setInputCloud(roomCloud);
    segment.setTargetCloud(metaroomCloud);
    segment.setDistanceThreshold(0.001);
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (metaroomCloud);
    segment.setSearchMethod(tree);
    segment.segment(*difference);
    ROS_INFO_STREAM("Computed differences");
    ROS_INFO_STREAM("Metaroom cloud "<<metaroomCloud->points.size()<<"  room cloud "<<roomCloud->points.size());

    if (difference->points.size() == 0)
    {
        // metaroom and room observation are identical -> no dynamic clusters can be computed

    ROS_INFO_STREAM("No dynamic clusters.");
        return;
    }

    std::vector<CloudPtr> vClusters = MetaRoom<PointType>::clusterPointCloud(difference,0.05,65,10000);
    metaroom->filterClustersBasedOnDistance(vClusters,3.0);

    ROS_INFO_STREAM("Clustered differences. "<<vClusters.size()<<" different clusters.");

    // combine clusters into one point cloud for publishing.
//    int colorId = 0;
//    CloudPtr dynamicClusters(new Cloud());
//    for (size_t i=0; i<vClusters.size(); i++)
//    {
//        for (size_t j=0; j<vClusters[i]->points.size(); j++)
//        {
//            uint8_t r = 0, g = 0, b = 0;
//            if (colorId == 0)
//            {
//                b = 255;
//            }
//            if (colorId == 1)
//            {
//                g = 255;
//            }

//            if (colorId == 2)
//            {
//                r = 127;
//                b = 127;
//            }
//            if (colorId == 3)
//            {
//                r = 255;
//                g = 102;
//                b = 51;
//            }
//            if (colorId == 4)
//            {
//                r = 255;
//                g = 51;
//                b = 204;
//            }
//            if (colorId == 5)
//            {
//                g = 138;
//                b = 184;
//            }

//            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//            vClusters[i]->points[j].rgb = *reinterpret_cast<float*>(&rgb);
//        }
//        colorId++;
//        if (colorId == 6)
//        {
//            colorId = 0;
//        }

//        *dynamicClusters += *vClusters[i];
//    }

//    CloudPtr dynamicClusters(new Cloud());
//    for (size_t i=0; i<vClusters.size(); i++)
//    {
//        *dynamicClusters += *vClusters[i];
//    }

    // Check cluster planarity and discard the absolutely planar ones (usually parts of walls, floor, ceiling).
    CloudPtr dynamicClusters(new Cloud());
    for (size_t i=0; i<vClusters.size(); i++)
    {
        pcl::SACSegmentation<PointType> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);
//        seg.setAxis(Eigen::Vector3f(0,0,1));
//        seg.setEpsAngle (0.1);

        seg.setInputCloud (vClusters[i]);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () > 0.9 * vClusters[i]->points.size())
        {
            ROS_INFO_STREAM("Discarding planar dynamic cluster");
        } else {
            *dynamicClusters += *vClusters[i];
        }
    }

//    // Filter out ceiling and floor clusters
//    pcl::PassThrough<PointType> lower_pass;
//    CloudPtr temp_cloud (new Cloud);
//    lower_pass.setInputCloud (dynamicClusters);
//    lower_pass.setFilterFieldName ("z");
//    lower_pass.setFilterLimits (-.5, .5);
////    lower_pass.filter (*temp_cloud);
////    *dynamicClusters = *temp_cloud;

//    pcl::PassThrough<PointType> upper_pass;
//    upper_pass.setInputCloud (dynamicClusters);
//    upper_pass.setFilterFieldName ("z");
//    upper_pass.setFilterLimits (0.1, 2.5);
//    upper_pass.filter (*temp_cloud);
//    *dynamicClusters = *temp_cloud;
    // cluster planarity

    // publish dynamic clusters
    sensor_msgs::PointCloud2 msg_clusters;
    pcl::toROSMsg(*dynamicClusters, msg_clusters);
    msg_clusters.header.frame_id="/map";
    m_PublisherDynamicClusters.publish(msg_clusters);
    ROS_INFO_STREAM("Published differences "<<dynamicClusters->points.size());

    aRoom.setDynamicClustersCloud(dynamicClusters);
    // save updated room
    parser.saveRoomAsXML(aRoom);

    if (aRoom.getRoomStringId() != "") // waypoint id set, update the map
    {
        m_WaypointToDynamicClusterMap[aRoom.getRoomStringId()] = dynamicClusters;
        ROS_INFO_STREAM("Updated map with new dynamic clusters for waypoint "<<aRoom.getRoomStringId());
    }

    if (m_bLogToDB)
    {
        QString databaseName = QString(aRoom.getRoomLogName().c_str()) + QString("/room_")+ QString::number(aRoom.getRoomRunNumber()) +QString("/dynamic_clusters");
        std::string id(m_messageStore.insertNamed(databaseName.toStdString(), msg_clusters));
        ROS_INFO_STREAM("Dynamic clusters \""<<databaseName.toStdString()<<"\" inserted with id "<<id);

        std::vector< boost::shared_ptr<sensor_msgs::PointCloud2> > results;
        if(m_messageStore.queryNamed<sensor_msgs::PointCloud2>(databaseName.toStdString(), results)) {

            BOOST_FOREACH( boost::shared_ptr<sensor_msgs::PointCloud2> p,  results)
            {
                CloudPtr databaseCloud(new Cloud());
                pcl::fromROSMsg(*p,*databaseCloud);
                ROS_INFO_STREAM("Got pointcloud by name. No points: " << databaseCloud->points.size());
            }
        }
    }
}


template <class PointType>
void SemanticMapNode<PointType>::roomObservationCallback(const semantic_map::RoomObservationConstPtr& obs_msg)
{
    std::cout<<"Room obs message received"<<std::endl;
    this->processRoomObservation(obs_msg->xml_file_name);
}

template <class PointType>
bool SemanticMapNode<PointType>::metaroomServiceCallback(MetaroomServiceRequest &req, MetaroomServiceResponse &res)
{
    ROS_INFO_STREAM("Received a metaroom request for waypoint "<<req.waypoint_id);

    WaypointMetaroomMapIterator it;
    it = m_WaypointToMetaroomMap.find(req.waypoint_id);

    if (it != m_WaypointToMetaroomMap.end())
    {
        ROS_INFO_STREAM("Metaroom found, it will be published on the /local_metric_map/metaroom topic.");
        res.reply = "OK";

        CloudPtr metaroomCloud = m_WaypointToMetaroomMap[req.waypoint_id]->getInteriorRoomCloud();
        sensor_msgs::PointCloud2 msg_metaroom;
        pcl::toROSMsg(*metaroomCloud, msg_metaroom);
        msg_metaroom.header.frame_id="/map";
        m_PublisherMetaroom.publish(msg_metaroom);

        return true;
    } else
    {
        ROS_INFO_STREAM("Metaroom not found, it will not be published.");
        res.reply = "failure";
        return false;
    }
}

template <class PointType>
bool SemanticMapNode<PointType>::observationServiceCallback(ObservationServiceRequest &req, ObservationServiceResponse &res)
{
    ROS_INFO_STREAM("Received an observation request for waypoint "<<req.waypoint_id);

    WaypointRoomMapIterator it;
    it = m_WaypointToRoomMap.find(req.waypoint_id);

    if (it != m_WaypointToRoomMap.end())
    {
        ROS_INFO_STREAM("Observation found, it will be published on the local_metric_map/merged_point_cloud_downsampled topic.");
        res.reply = "OK";

        CloudPtr observationCloud = m_WaypointToRoomMap[req.waypoint_id].getInteriorRoomCloud();
        sensor_msgs::PointCloud2 msg_observation;
        pcl::toROSMsg(*observationCloud, msg_observation);
        msg_observation.header.frame_id="/map";
        m_PublisherObservation.publish(msg_observation);

        return true;
    } else
    {
        ROS_INFO_STREAM("Observation not found, it will not be published.");
        res.reply = "failure";
        return false;
    }
}

template <class PointType>
bool SemanticMapNode<PointType>::dynamicClusterServiceCallback(DynamicClusterServiceRequest &req, DynamicClusterServiceResponse &res)
{
    ROS_INFO_STREAM("Received a dynamic clusters request for waypoint "<<req.waypoint_id);

    WaypointPointCloudMapIterator it;
    it = m_WaypointToDynamicClusterMap.find(req.waypoint_id);

    if (it != m_WaypointToDynamicClusterMap.end())
    {
        ROS_INFO_STREAM("Dynamic clusters found, it will be published on the /local_metric_map/dynamic_clusters topic.");
        res.reply = "OK";

        // publish dynamic clusters
        sensor_msgs::PointCloud2 msg_clusters;
        pcl::toROSMsg(*m_WaypointToDynamicClusterMap[req.waypoint_id], msg_clusters);
        msg_clusters.header.frame_id="/map";
        m_PublisherDynamicClusters.publish(msg_clusters);

        return true;
    } else
    {
        ROS_INFO_STREAM("Dynamic clusters not found, it will not be published.");
        res.reply = "failure";
        return false;
    }
}


#endif
