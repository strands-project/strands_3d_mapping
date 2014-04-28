#ifndef __CLOUD_MERGE_NODE__H
#define __CLOUD_MERGE_NODE__H

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
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <semantic_map/RoomObservation.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "cloud_merge.h"
#include "room.h"
#include "roomXMLparser.h"
#include "semanticMapSummaryParser.h"

template <class PointType>
class CloudMergeNode {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    CloudMergeNode(ros::NodeHandle nh);
    ~CloudMergeNode();

    void controlCallback(const std_msgs::String& controlString);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);


    ros::Subscriber                                                             m_SubscriberControl;
    ros::Subscriber                                                             m_SubscriberPointCloud;
    ros::Publisher                                                              m_PublisherMergedCloud;
    ros::Publisher                                                              m_PublisherIntermediateCloud;   
    ros::Publisher                                                              m_PublisherRoomObservation;

    image_transport::Publisher                                                              m_RosPublisherIntermediateRGB;
    image_transport::Publisher                                                  m_RosPublisherIntermediateDepth;
    ros::Publisher                                                              m_RosPublisherIntermediateDepthCamInfo;
    ros::Publisher                                                              m_RosPublisherIntermediateRGBCamInfo;

    boost::shared_ptr<image_transport::ImageTransport>                          m_ImageTransport;

    boost::shared_ptr<Synchronizer>                                             m_MessageSynchronizer;
    image_transport::SubscriberFilter                                           m_DepthSubscriber, m_RGBSubscriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo>                        m_CameraInfoSubscriber;
    boost::shared_ptr<image_transport::ImageTransport>                          m_RGB_it, m_Depth_it;

private:
    ros::NodeHandle                                                             m_NodeHandle;
    CloudMerge<PointType>                                                       m_CloudMerge;

    tf::TransformListener                                                       m_TransformListener;
    bool                                                                        m_bAquisitionPhase;

    bool                                                                        m_bAquireData;

    int                                                                         scan_skip_counter;

    std::string                                                                 m_LogRunName;
    bool                                                                        m_LogNameInitialized;
    std::string                                                                 m_LogName;
    SemanticRoom<PointType>                                                     aSemanticRoom;
    int                                                                         m_SemanticRoomId;

    bool                                                                        m_bUseImages;
    bool                                                                        m_bSaveIntermediateData;

};

template <class PointType>
CloudMergeNode<PointType>::CloudMergeNode(ros::NodeHandle nh) : m_TransformListener(nh,ros::Duration(1000))
{
    ROS_INFO_STREAM("Cloud merge node initialized");

    m_NodeHandle = nh;
    m_RGB_it.reset(new image_transport::ImageTransport(nh));
    m_Depth_it.reset(new image_transport::ImageTransport(nh));

    m_MessageSynchronizer.reset( new Synchronizer(SyncPolicy(15), m_DepthSubscriber, m_RGBSubscriber, m_CameraInfoSubscriber) );
    m_MessageSynchronizer->registerCallback(boost::bind(&CloudMergeNode::imageCallback, this, _1, _2, _3));

    m_SubscriberControl = m_NodeHandle.subscribe("/ptu/log",1, &CloudMergeNode::controlCallback,this);

    m_PublisherIntermediateCloud = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/intermediate_point_cloud", 1);
    m_PublisherMergedCloud = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/merged_point_cloud", 1);

    // intermediate image publishers (use image transport for compressed images).
    m_ImageTransport = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));
    m_RosPublisherIntermediateDepth = m_ImageTransport->advertise("/local_metric_map/depth/depth_filtered", 1000);
    m_RosPublisherIntermediateRGB = m_ImageTransport->advertise("/local_metric_map/rgb/rgb_filtered", 1000);
    m_RosPublisherIntermediateDepthCamInfo = m_NodeHandle.advertise<sensor_msgs::CameraInfo>("/local_metric_map/depth/camera_info", 1000);   
    m_RosPublisherIntermediateRGBCamInfo = m_NodeHandle.advertise<sensor_msgs::CameraInfo>("/local_metric_map/rgb/camera_info", 1000);

    // room publisher
    m_PublisherRoomObservation = m_NodeHandle.advertise<semantic_map::RoomObservation>("/local_metric_map/room_observations", 1000);

    image_transport::TransportHints hints("raw", ros::TransportHints(), m_NodeHandle);

    // subscribe to input clouds


    bool generate_pointclouds;
    m_NodeHandle.param<bool>("generate_pointclouds",generate_pointclouds, true);

    if (!generate_pointclouds)
    {
        m_bUseImages = false;
        ROS_INFO_STREAM("Not generating point clouds, using them directly from the camera.");
    } else {
        m_bUseImages = true;
        ROS_INFO_STREAM("Generating point clouds from camera RGBD images.");
    }

    if (!m_bUseImages)
    {
        // subscribe to point cloud channels
        std::string cloud_channel;
        bool found = m_NodeHandle.getParam("input_cloud",cloud_channel);
        if (!found)
        {
            cloud_channel = "/depth_registered/points";
        }
        m_SubscriberPointCloud = m_NodeHandle.subscribe(cloud_channel,1, &CloudMergeNode<pcl::PointXYZRGB>::pointCloudCallback,this);

        ROS_INFO_STREAM("Subscribed to point cloud topic "<<cloud_channel);
    } else {
        // subscribe to image channels
        std::string depth_channel;
        bool found = m_NodeHandle.getParam("input_depth",depth_channel);
        if (!found)
        {
            depth_channel = "/head_xtion/depth_registered/image_rect";
        }
        m_DepthSubscriber.subscribe(*m_Depth_it, depth_channel,       1);
        ROS_INFO_STREAM("Subscribed to depth image topic "<<depth_channel);

        std::string rgb_channel;
        found = m_NodeHandle.getParam("input_rgb",rgb_channel);
        if (!found)
        {
            rgb_channel = "/head_xtion/rgb/image_color";
        }
        m_RGBSubscriber.subscribe(*m_RGB_it, rgb_channel,       1);
        ROS_INFO_STREAM("Subscribed to rgb image topic "<<rgb_channel);

        std::string caminfo_channel;
        found = m_NodeHandle.getParam("input_caminfo",caminfo_channel);
        if (!found)
        {
            caminfo_channel = "/head_xtion/rgb/camera_info";
        }
        m_CameraInfoSubscriber.subscribe(m_NodeHandle, caminfo_channel,       1);
        ROS_INFO_STREAM("Subscribed to camera info topic "<<caminfo_channel);

    }





    m_bAquireData = false;
    m_bAquisitionPhase = false;

    scan_skip_counter = 0;

    bool found = m_NodeHandle.getParam("/log_run_name",m_LogRunName);
    if (!found)
    {
        std::cout<<"Parameter \"/log_run_name\" hasn't been defined. Defaulting to patrol_run_1"<<std::endl;
        m_LogRunName = "patrol_run_1";
    }

    found = m_NodeHandle.getParam("save_intermediate",m_bSaveIntermediateData);
    if (!m_bSaveIntermediateData)
    {
        ROS_INFO_STREAM("Not saving intermediate data.");
    } else {
        ROS_INFO_STREAM("Saving intermediate data.");
    }

    bool doCleanup=true;
    m_NodeHandle.param<bool>("cleanup",doCleanup,true);
    if (!doCleanup)
    {
        ROS_INFO_STREAM("Not removing old data.");
    } else {
        ROS_INFO_STREAM("Will remove old data.");
        SemanticMapSummaryParser<PointType> summaryParser;
        summaryParser.removeSemanticMapData();
    }

    m_LogNameInitialized = false;
    m_LogName = "";
    m_SemanticRoomId = -1;

}

template <class PointType>
CloudMergeNode<PointType>::~CloudMergeNode()
{

}

template <class PointType>
void CloudMergeNode<PointType>::imageCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{

    if (!m_bUseImages)
    {
        return; // we don't need to process images
    }

//    ROS_INFO_STREAM("image callback");

    if (m_bAquireData && m_bAquisitionPhase)
    {
        scan_skip_counter++;
        if (scan_skip_counter < 7)
            return;

        m_CloudMerge.addIntermediateImage(depth_msg, rgb_msg, info_msg);
    }

}



template <class PointType>
void CloudMergeNode<PointType>::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (m_bUseImages)
    {
        return; // we don't need to process images
    }

    ROS_INFO_STREAM("Point cloud callback");
    if (m_bAquireData && m_bAquisitionPhase)
    {
        scan_skip_counter++;
        if (scan_skip_counter < 7)
            return;

        CloudPtr new_cloud(new Cloud());
        pcl::fromROSMsg(*msg, *new_cloud);

        new_cloud->header = pcl_conversions::toPCL(msg->header);

        m_CloudMerge.addIntermediateCloud(*new_cloud);
    }
}

template <class PointType>
void CloudMergeNode<PointType>::controlCallback(const std_msgs::String& controlString)
{
//    ROS_INFO_STREAM("Received control string "<<controlString);

    if (controlString.data == "start_sweep")
    {
        ROS_INFO_STREAM("Pan tilt sweep started -> acquire data");
        m_bAquisitionPhase = true;

        // initialize log name
        {
            // check if the run name has been defined
            std::string new_log_name;
            bool found = m_NodeHandle.getParam("/log_run_name",new_log_name);
            if (!found)
            {
                std::cout<<"Parameter \"/log_run_name\" hasn't been defined. Defaulting to patrol_run_1"<<std::endl;
                m_LogRunName = "patrol_run_1";
            } else {
                if (new_log_name != m_LogRunName)
                {
                    m_SemanticRoomId=-1; // reset room id as we've started a new run
                }
                m_LogRunName = new_log_name;
            }

            static std::locale loc(std::cout.getloc(),
                                   new boost::posix_time::time_facet("%Y%m%d"));

            std::basic_stringstream<char> wss;
            wss.imbue(loc);
            wss << ros::Time::now().toBoost();
            m_LogName = wss.str()+"_"+m_LogRunName;
            ROS_INFO_STREAM("Log name set to "<<m_LogName);
        }

         // initialize room
         m_SemanticRoomId++;
//         m_vSemanticRoom.push_back(SemanticRoom<PointType>());
         aSemanticRoom.setSaveIntermediateClouds(m_bSaveIntermediateData);
         aSemanticRoom.setRoomRunNumber(m_SemanticRoomId);
         aSemanticRoom.setRoomLogName(m_LogName);

         // get room start time
         aSemanticRoom.setRoomLogStartTime(ros::Time::now().toBoost());
         aSemanticRoom.clearIntermediateClouds();
         aSemanticRoom.resetRoomTransform();

         m_CloudMerge.resetIntermediateCloud();
         m_CloudMerge.resetMergedCloud();

    }

    if (controlString.data == "end_sweep")
    {
        ROS_INFO_STREAM("Pan tilt sweep stopped");
        m_bAquisitionPhase = false;

        // publish the merged cloud
        m_CloudMerge.subsampleMergedCloud(0.01f,0.01f,0.01f);
        CloudPtr merged_cloud = m_CloudMerge.getMergedCloud();

        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(*merged_cloud, msg_cloud);
        m_PublisherMergedCloud.publish(msg_cloud);

        // set room end time
        aSemanticRoom.setRoomLogEndTime(ros::Time::now().toBoost());
        // add complete cloud to semantic room
        aSemanticRoom.setCompleteRoomCloud(merged_cloud);

        SemanticRoomXMLParser<PointType> parser;
        ROS_INFO_STREAM("Saving semantic room file");
        std::string roomXMLPath = parser.saveRoomAsXML(aSemanticRoom);
        ROS_INFO_STREAM("Saved semantic room");

        // Pulbish room observation
        semantic_map::RoomObservation obs_msg;
        obs_msg.xml_file_name = roomXMLPath;

        m_PublisherRoomObservation.publish(obs_msg);
    }

    if ((controlString.data == "start_position") && m_bAquisitionPhase)
    {
        //ROS_INFO_STREAM("Acquire data at static position");
        m_bAquireData = true;
        // sleep for 0.5 seconds to make sure the pan tilt stopped moving
//        ros::Rate loop_rate(500);
//        loop_rate.sleep();
        scan_skip_counter = 0;

    }

    if ((controlString.data == "end_position") && m_bAquisitionPhase)
    {
       // ROS_INFO_STREAM("Stop aquiring data");
        m_bAquireData = false;

        // process the accumulated point cloud
        CloudPtr transformed_cloud = m_CloudMerge.subsampleIntermediateCloud();
        if (transformed_cloud->points.size() == 0)
        {
            std::cout<<"Intermediate cloud has zero points. Skipping."<<std::endl;
            return;
        }


        try{
            tf::StampedTransform transform;

            sensor_msgs::PointCloud2 temp_msg;
            pcl::toROSMsg(*transformed_cloud, temp_msg);
            temp_msg.header = pcl_conversions::fromPCL(transformed_cloud->header);
            m_PublisherIntermediateCloud.publish(temp_msg); // publish in the local frame of reference

            m_TransformListener.waitForTransform("/map", transformed_cloud->header.frame_id,temp_msg.header.stamp, ros::Duration(20.0) );
            m_TransformListener.lookupTransform("/map", transformed_cloud->header.frame_id,
                                     temp_msg.header.stamp, transform);

            m_CloudMerge.transformIntermediateCloud(transform,"/map");

            transformed_cloud = m_CloudMerge.getIntermediateCloud();

            static int cloud_id = 0;

            // add intermediat cloud and transform to semantic room
            aSemanticRoom.addIntermediateRoomCloud(transformed_cloud, transform);


            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*transformed_cloud, msg_cloud);

            m_RosPublisherIntermediateDepth.publish(m_CloudMerge.m_IntermediateFilteredDepthImage);
            m_RosPublisherIntermediateRGB.publish(m_CloudMerge.m_IntermediateFilteredRGBImage);
            m_RosPublisherIntermediateDepthCamInfo.publish(m_CloudMerge.m_IntermediateFilteredDepthCamInfo);

//            ROS_INFO_STREAM("Transformed cloud aquisition time "<<transformed_cloud->header.stamp<<"  and frame "<<transformed_cloud->header.frame_id<<"  and points "<<transformed_cloud->points.size());

        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        m_CloudMerge.processIntermediateCloud();
    }
}



#endif
