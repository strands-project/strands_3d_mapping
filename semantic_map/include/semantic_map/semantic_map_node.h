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
//#include <semantic_map/MetaroomService.h>
//#include <semantic_map/DynamicClusterService.h>
//#include <semantic_map/ObservationService.h>

#include <semantic_map/RoomObservation.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "room_xml_parser.h"
#include "metaroom_xml_parser.h"
#include "semantic_map_summary_parser.h"
#include "reg_features.h"
#include "reg_transforms.h"
#include "room_utilities.h"
#include "load_utilities.h"

#include <strands_sweep_registration/RobotContainer.h>


template <class PointType>
class SemanticMapNode {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename SemanticMapSummaryParser::EntityStruct Entities;
    typedef pcl::search::KdTree<PointType> Tree;

    typedef typename std::map<std::string, boost::shared_ptr<MetaRoom<PointType> > >::iterator WaypointMetaroomMapIterator;
    typedef typename std::map<std::string, SemanticRoom<PointType> >::iterator WaypointRoomMapIterator;
    typedef typename std::map<std::string, CloudPtr>::iterator WaypointPointCloudMapIterator;

    SemanticMapNode(ros::NodeHandle nh);
    ~SemanticMapNode();

    void processRoomObservation(std::string xml_file_name);

    void roomObservationCallback(const semantic_map::RoomObservationConstPtr& obs_msg);

    ros::Subscriber                                                             m_SubscriberRoomObservation;
    ros::Publisher                                                              m_PublisherMetaroom;
    ros::Publisher                                                              m_PublisherObservation;
    ros::Publisher                                                              m_PublisherDynamicClusters;
    ros::Publisher                                                              m_PublisherStatus;

private:

    ros::NodeHandle                                                             m_NodeHandle;
    SemanticMapSummaryParser                                         m_SummaryParser;
    bool                                                                        m_bSaveIntermediateData;
    std::vector<boost::shared_ptr<MetaRoom<PointType> > >                       m_vLoadedMetarooms;
    mongodb_store::MessageStoreProxy                                           m_messageStore;
    bool                                                                        m_bLogToDB;
    std::map<std::string, boost::shared_ptr<MetaRoom<PointType> > >             m_WaypointToMetaroomMap;
    std::map<std::string, SemanticRoom<PointType> >                             m_WaypointToRoomMap;
    std::map<std::string, CloudPtr>                                             m_WaypointToDynamicClusterMap;
    bool                                                                        m_bUpdateMetaroom;
    bool                                                                        m_bNewestClusters;

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

    m_NodeHandle.param<bool>("newest_dynamic_clusters",m_bNewestClusters,false);
    if (m_bNewestClusters)
    {
        ROS_INFO_STREAM("Only the latest dynamic clusters will be computed (comapring with previous observations).");
    } else {
        ROS_INFO_STREAM("All the dynamic clusters will be computer (comparing with the metaroom).");
    }

    std::string statusTopic;
    m_NodeHandle.param<std::string>("status_topic",statusTopic,"/local_metric_map/status");
    ROS_INFO_STREAM("The status topic is "<<statusTopic);
    m_PublisherStatus = m_NodeHandle.advertise<std_msgs::String>(statusTopic, 1000);
}

template <class PointType>
SemanticMapNode<PointType>::~SemanticMapNode()
{

}


template <class PointType>
void SemanticMapNode<PointType>::processRoomObservation(std::string xml_file_name)
{
    std::cout<<"File name "<<xml_file_name<<std::endl;

    if ( ! boost::filesystem::exists( xml_file_name ) )
    {
        ROS_ERROR_STREAM("Xml file does not exist. Aborting.");
        std_msgs::String msg;
        msg.data = "error_processing_observation";
        m_PublisherStatus.publish(msg);
        return;
    }

    SemanticRoomXMLParser<PointType> parser;
    SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(xml_file_name,true);
    aRoom.resetRoomTransform();

    // update summary xml
    m_SummaryParser.createSummaryXML<PointType>();

    // update list of rooms & metarooms
    m_SummaryParser.refresh();

    ROS_INFO_STREAM("Summary XML created.");

    boost::shared_ptr<MetaRoom<PointType> > metaroom;
    bool found = false;

    std::string matchingMetaroomXML = "";
    // first check if the matching metaroom has already been loaded
    for (size_t i=0; i<m_vLoadedMetarooms.size(); i++)
    {
        // compare by waypoint, if set
        if (aRoom.getRoomStringId() != "")
        {
            if (aRoom.getRoomStringId() == m_vLoadedMetarooms[i]->m_sMetaroomStringId)
            {
                metaroom = m_vLoadedMetarooms[i];
                found = true;
                break;
            }
        } else {
            // if not set, compare by centroid
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
    }

    // if not loaded already, look through already saved metarooms
    if (!found)
    {
        std::vector<Entities> allMetarooms = m_SummaryParser.getMetaRooms();
        ROS_INFO_STREAM("Loaded "<<allMetarooms.size()<<" metarooms.");
        for (size_t i=0; i<allMetarooms.size();i++)
        {
            // compare by waypoint, if set
            if (aRoom.getRoomStringId() != "")
            {
                if (aRoom.getRoomStringId() == allMetarooms[i].stringId)
                {
                    matchingMetaroomXML = allMetarooms[i].roomXmlFile;
                    break;
                }
            } else {
                // if not set, compare by centroid
                double centroidDistance = pcl::distances::l2(allMetarooms[i].centroid,aRoom.getCentroid());
                if (! (centroidDistance < ROOM_CENTROID_DISTANCE) )
                {
                    continue;
                } else {
                    matchingMetaroomXML = allMetarooms[i].roomXmlFile;
                    break;
                }
            }
        }

        if (matchingMetaroomXML == "")
        {
            ROS_INFO_STREAM("No matching metaroom found. Create new metaroom.");
            metaroom =  boost::shared_ptr<MetaRoom<PointType> >(new MetaRoom<PointType>());

        } else {
            ROS_INFO_STREAM("Matching metaroom found. XML file: "<<matchingMetaroomXML);
            metaroom =  boost::shared_ptr<MetaRoom<PointType> >(new MetaRoom<PointType>(MetaRoomXMLParser<PointType>::loadMetaRoomFromXML(matchingMetaroomXML,false)));
            found = true;
        }
    } else {
        MetaRoomXMLParser<PointType> meta_parser;
        matchingMetaroomXML= meta_parser.findMetaRoomLocation(metaroom.get()).toStdString();
        matchingMetaroomXML+="/metaroom.xml";
    }

    metaroom->setUpdateMetaroom(m_bUpdateMetaroom);
    metaroom->setSaveIntermediateSteps(m_bSaveIntermediateData);

    // update metaroom
    // check if the sweep poses have been precomputed
    unsigned int x,y;
    double*** rawPoses = semantic_map_registration_transforms::loadRegistrationTransforms(x,y);
    RegistrationFeatures reg(false);
    std::vector<RegistrationFeatures::RegistrationData> mr_features = reg.loadOrbFeatures(matchingMetaroomXML, false);
    std::vector<RegistrationFeatures::RegistrationData> room_features = reg.loadOrbFeatures(xml_file_name, false);
    std::cout<<"MR features "<<mr_features.size()<<" room features "<<room_features.size()<<std::endl;
    std::cout<<"Room xml "<<xml_file_name<<" MR xml "<<matchingMetaroomXML<<std::endl;

    bool do_registration = true;
    if (!found)
    {
        ROS_INFO_STREAM("Initializing metaroom.");
    } else {

        if ((rawPoses !=NULL) && (mr_features.size() != 0) && (room_features.size() != 0))
        {
            ROS_INFO_STREAM("Registering sweep using ORB features.");
            unsigned int gx = 17;
            unsigned int todox = 17;
            unsigned int gy = 3;
            unsigned int todoy = 3;
            RobotContainer* new_rc = new RobotContainer(gx,todox,gy,todoy);
            new_rc->initializeCamera(540.0, 540.0,319.5, 219.5, 640, 480);
            for (size_t i=0; i<todox; i++){
                for (size_t j=0; j<todoy;j++){
                    for (size_t k=0; k<6;k++)
                    {

                        new_rc->poses[i][j][k] = rawPoses[i][j][k];
                    }
                }
            }
            new_rc->addToTrainingORBFeatures(matchingMetaroomXML);
            new_rc->addToTrainingORBFeatures(xml_file_name);
            std::vector<Eigen::Matrix4f> reg_transforms = new_rc->alignAndStoreSweeps();

            std::vector<Eigen::Matrix4f> sweepPoses = new_rc->sweeps[1]->getPoseVector();
            auto roomTransforms = aRoom.getIntermediateCloudTransforms();
            aRoom.clearIntermediateCloudRegisteredTransforms();

            for (size_t j=0; j<roomTransforms.size();j++)
            {
                tf::StampedTransform roomTransform = roomTransforms[j];
                const Eigen::Affine3d eigenTr(sweepPoses[j].cast<double>());
                tf::Transform tfTr;
                tf::transformEigenToTF(eigenTr, tfTr);

                roomTransform.setOrigin(tfTr.getOrigin());
                roomTransform.setBasis(tfTr.getBasis());

                aRoom.addIntermediateRoomCloudRegisteredTransform(roomTransform);
            }

            semantic_map_room_utilities::rebuildRegisteredCloud<PointType>(aRoom);
            CloudPtr roomCloud = aRoom.getCompleteRoomCloud();
            pcl::transformPointCloud (*roomCloud, *roomCloud, metaroom->getRoomTransform());

            aRoom.setCompleteRoomCloud(roomCloud);

            // update room transform
            tf::StampedTransform sweep_to_map = aRoom.getIntermediateCloudTransforms()[0];
            Eigen::Affine3d eigen_affine; tf::transformTFToEigen(sweep_to_map, eigen_affine);
            Eigen::Matrix4f eigen_matrix(eigen_affine.matrix().cast<float>());
            aRoom.setRoomTransform(metaroom->getRoomTransform() * eigen_matrix.inverse());

            SemanticRoomXMLParser<PointType> parser;
            parser.saveRoomAsXML(aRoom);
            delete new_rc;
            do_registration = false;
        } else {
            ROS_INFO_STREAM("Preregistered sweep poses not found or ORB features not computed. Registering with NDT.");
        }
    }

    auto updateIteration = metaroom->updateMetaRoom(aRoom,"",do_registration);

    // save metaroom
    ROS_INFO_STREAM("Saving metaroom.");
    MetaRoomXMLParser<PointType> meta_parser;
    meta_parser.saveMetaRoomAsXML(*metaroom);

    if (updateIteration.roomRunNumber == -1) // no update performed
    {
        ROS_ERROR_STREAM("Could not update metaroom with the room instance.");
        std_msgs::String msg;
        msg.data = "error_processing_observation";
        m_PublisherStatus.publish(msg);
        return;
    }

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
        m_WaypointToRoomMap[aRoom.getRoomStringId()] = aRoom;
    }


    CloudPtr roomCloud = aRoom.getInteriorRoomCloud();
    ROS_INFO_STREAM("Computing differences");
    CloudPtr difference(new Cloud());
    // compute differences
    if (!m_bNewestClusters)
    {
        pcl::SegmentDifferences<PointType> segment;
        segment.setInputCloud(roomCloud);
        segment.setTargetCloud(metaroomCloud);
        segment.setDistanceThreshold(0.001);
        typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (metaroomCloud);
        segment.setSearchMethod(tree);
        segment.segment(*difference);
        ROS_INFO_STREAM("Computed differences " << difference->points.size());
        ROS_INFO_STREAM("Comparison cloud "<<metaroomCloud->points.size()<<"  room cloud "<<roomCloud->points.size());
    } else {
        // compare with previous observation

        // look for the previous observation in ~/.semanticMap/
        passwd* pw = getpwuid(getuid());
        std::string dataPath(pw->pw_dir);
        dataPath+="/.semanticMap";

        std::vector<std::string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(dataPath, aRoom.getRoomStringId());
        if (matchingObservations.size() == 0)
        {
            ROS_ERROR_STREAM("No observations for this waypoint saved on the disk "+aRoom.getRoomStringId()+" cannot compare with previous observation.");
            std_msgs::String msg;
            msg.data = "finished_processing_observation";
            m_PublisherStatus.publish(msg);
            return;
        }
        if (matchingObservations.size() == 1)
        {
            ROS_INFO_STREAM("No dynamic clusters.");
            std_msgs::String msg;
            msg.data = "finished_processing_observation";
            m_PublisherStatus.publish(msg);
            return;
        }

        sort(matchingObservations.begin(), matchingObservations.end());
        reverse(matchingObservations.begin(), matchingObservations.end());
        std::string latest = matchingObservations[0];
        if (latest != xml_file_name)
        {
            ROS_WARN_STREAM("The xml file for the latest observations "+latest+" is different from the one provided "+xml_file_name+" Aborting");
        }

        std::string previous = matchingObservations[1];
        ROS_INFO_STREAM("Comparing with "<<previous);
        SemanticRoomXMLParser<PointType> parser;
        SemanticRoom<PointType> prevRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(previous,true);

        CloudPtr prevCloud = prevRoom.getInteriorRoomCloud();

        // compute differences and check occlusions
        CloudPtr differenceRoomToPrevRoom(new Cloud);
        CloudPtr differencePrevRoomToRoom(new Cloud);

        // compute the differences
        pcl::SegmentDifferences<PointType> segment;
        segment.setInputCloud(roomCloud);
        segment.setTargetCloud(prevCloud);
        segment.setDistanceThreshold(0.001);
        typename Tree::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (prevCloud);
        segment.setSearchMethod(tree);
        segment.segment(*differenceRoomToPrevRoom);

        segment.setInputCloud(prevCloud);
        segment.setTargetCloud(roomCloud);
        tree->setInputCloud(roomCloud);

        segment.segment(*differencePrevRoomToRoom);

        CloudPtr toBeAdded(new Cloud());
        CloudPtr toBeRemoved(new Cloud());

        OcclusionChecker<PointType> occlusionChecker;
        occlusionChecker.setSensorOrigin(metaroom->getSensorOrigin()); // since it's already transformed in the metaroom frame of ref
        auto occlusions = occlusionChecker.checkOcclusions(differenceRoomToPrevRoom,differencePrevRoomToRoom, 720 );
        *difference = *occlusions.toBeRemoved;

    }




    if (difference->points.size() == 0)
    {
        // metaroom and room observation are identical -> no dynamic clusters can be computed

        ROS_INFO_STREAM("No dynamic clusters.");
        std_msgs::String msg;
        msg.data = "finished_processing_observation";
        m_PublisherStatus.publish(msg);
        return;
    }


    std::vector<CloudPtr> vClusters = MetaRoom<PointType>::clusterPointCloud(difference,0.03,800,100000);
    ROS_INFO_STREAM("Clustered differences. "<<vClusters.size()<<" different clusters.");
    MetaRoom<PointType>::filterClustersBasedOnDistance(aRoom.getIntermediateCloudTransforms()[0].getOrigin(), vClusters,3.0);
    ROS_INFO_STREAM(vClusters.size()<<" different clusters after max distance filtering.");

    ROS_INFO_STREAM("Clustered differences. "<<vClusters.size()<<" different clusters.");

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

        seg.setInputCloud (vClusters[i]);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () > 0.9 * vClusters[i]->points.size())
        {
            ROS_INFO_STREAM("Discarding planar dynamic cluster");
        } else {
            *dynamicClusters += *vClusters[i];
        }
    }

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

    std_msgs::String msg;
    msg.data = "finished_processing_observation";
    m_PublisherStatus.publish(msg);
    return;
}

template <class PointType>
void SemanticMapNode<PointType>::roomObservationCallback(const semantic_map::RoomObservationConstPtr& obs_msg)
{
    std::cout<<"Room obs message received"<<std::endl;
    this->processRoomObservation(obs_msg->xml_file_name);
}

#endif
