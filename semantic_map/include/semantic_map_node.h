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

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <semantic_map/RoomObservation.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

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

    SemanticMapNode(ros::NodeHandle nh);
    ~SemanticMapNode();

    void roomObservationCallback(const semantic_map::RoomObservationConstPtr& obs_msg);


    ros::Subscriber                                                             m_SubscriberRoomObservation;
    ros::Publisher                                                              m_PublisherMetaroom;
    ros::Publisher                                                              m_PublisherDynamicClusters;

private:
    ros::NodeHandle                                                             m_NodeHandle;
    SemanticMapSummaryParser<PointType>                                         m_SummaryParser;
    bool                                                                        m_bSaveIntermediateData;
    std::vector<boost::shared_ptr<MetaRoom<PointType> > >                       m_vLoadedMetarooms;
};

template <class PointType>
SemanticMapNode<PointType>::SemanticMapNode(ros::NodeHandle nh)
{
    ROS_INFO_STREAM("Semantic map node initialized");

    m_NodeHandle = nh;

    m_SubscriberRoomObservation = m_NodeHandle.subscribe("/local_metric_map/room_observations",1, &SemanticMapNode::roomObservationCallback,this);

    m_PublisherMetaroom = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/metaroom", 1);
    m_PublisherDynamicClusters = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/dynamic_clusters", 1);

    std::string save_intermediate;
    bool found = m_NodeHandle.getParam("save_intermediate",save_intermediate);
    if (found)
    {
        if (save_intermediate == "no")
        {
            m_bSaveIntermediateData = false;
            ROS_INFO_STREAM("Not saving intermediate data.");
        } else {
            m_bSaveIntermediateData = true;
            ROS_INFO_STREAM("Saving intermediate data.");
        }
    } else {
        ROS_INFO_STREAM("Parameter save_intermediate not defined. Defaulting to not saving the intermediate data.");
        m_bSaveIntermediateData = false;
    }
}

template <class PointType>
SemanticMapNode<PointType>::~SemanticMapNode()
{

}

template <class PointType>
void SemanticMapNode<PointType>::roomObservationCallback(const semantic_map::RoomObservationConstPtr& obs_msg)
{
    std::cout<<"Room obs message received"<<std::endl;
    std::cout<<"File name "<<obs_msg->xml_file_name<<std::endl;

    SemanticRoomXMLParser<PointType> parser;
    SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(obs_msg->xml_file_name,false);
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


    // compute differences
    CloudPtr difference(new Cloud());

    pcl::SegmentDifferences<PointType> segment;
    segment.setInputCloud(roomCloud);
    segment.setTargetCloud(metaroomCloud);
    segment.setDistanceThreshold(0.001);
    typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (metaroomCloud);
    segment.setSearchMethod(tree);
    segment.segment(*difference);

    if (difference->points.size() == 0)
    {
        // metaroom and room observation are identical -> no dynamic clusters can be computed
        return;
    }

    std::vector<CloudPtr> vClusters = MetaRoom<PointType>::clusterPointCloud(difference,0.05,500,100000);
    metaroom->filterClustersBasedOnDistance(vClusters,2.5);

    // combine clusters into one point cloud for publishing.
    CloudPtr dynamicClusters(new Cloud());
    for (size_t i=0; i<vClusters.size(); i++)
    {
        *dynamicClusters += *vClusters[i];
    }

    // publish dynamic clusters
    sensor_msgs::PointCloud2 msg_clusters;
    pcl::toROSMsg(*dynamicClusters, msg_clusters);
    msg_clusters.header.frame_id="/map";
    m_PublisherDynamicClusters.publish(msg_clusters);

}

#endif
