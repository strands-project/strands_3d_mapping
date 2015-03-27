#ifndef __CLOUD_MERGE_NODE__H
#define __CLOUD_MERGE_NODE__H

#include <stdio.h>
#include <iosfwd>
#include <stdlib.h>
#include <string>
#include <sys/types.h>
#include <pwd.h>
#include <boost/filesystem.hpp>


// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>

#include <mongodb_store/message_store.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// QT
#include <QFile>
#include <QDir>
#include <qtconcurrentrun.h>

// PCL includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/date_time/posix_time/posix_time.hpp>


#include <semantic_map/RoomObservation.h>
#include <semantic_map/room.h>
#include <semantic_map/room_xml_parser.h>
#include <semantic_map/semantic_map_summary_parser.h>
#include <semantic_map/reg_transforms.h>
#include <semantic_map/reg_features.h>
#include <semantic_map/room_utilities.h>
#include <semantic_map/mongodb_interface.h>

#include "cloud_merge.h"



template <class PointType>
class CloudMergeNode {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    typedef typename SemanticMapSummaryParser::EntityStruct Entities;

    CloudMergeNode(ros::NodeHandle nh);
    ~CloudMergeNode();

    void controlCallback(const std_msgs::String& controlString);
    void topoNodeCallback(const std_msgs::String& controlString);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, const sensor_msgs::CameraInfoConstPtr& info_msg_depth);

    void findSemanticRoomIDAndLogName(SemanticRoom<PointType>& aSemanticRoom, int& roomId, int& patrolNumber);
    void getRoomIDAndPatrolNumber(QString roomXmlFile, int& roomId, int& patrolNumber);


    ros::Subscriber                                                             m_SubscriberControl;
    ros::Subscriber                                                             m_SubscriberPointCloud;
    ros::Subscriber                                                             m_SubscriberTopoNode;
    ros::Publisher                                                              m_PublisherMergedCloud;
    ros::Publisher                                                              m_PublisherMergedCloudDownsampled;
    ros::Publisher                                                              m_PublisherIntermediateCloud;   
    ros::Publisher                                                              m_PublisherRoomObservation;

    image_transport::Publisher                                                  m_RosPublisherIntermediateRGB;
    image_transport::Publisher                                                  m_RosPublisherIntermediateDepth;
    ros::Publisher                                                              m_RosPublisherIntermediateDepthCamInfo;
    ros::Publisher                                                              m_RosPublisherIntermediateRGBCamInfo;

    boost::shared_ptr<image_transport::ImageTransport>                          m_ImageTransport;

    boost::shared_ptr<Synchronizer>                                             m_MessageSynchronizer;
    image_transport::SubscriberFilter                                           m_DepthSubscriber, m_RGBSubscriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo>                        m_RGBCameraInfoSubscriber;
    message_filters::Subscriber<sensor_msgs::CameraInfo>                        m_DepthCameraInfoSubscriber;
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
    int                                                                         m_MaxInstances;
    MongodbInterface                                                            m_MongodbInterface;

    bool                                                                        m_bLogToDB;
    bool                                                                        m_bCacheOldData;
    bool                                                                        m_bSaveIntermediateImages;
    std::string                                                                 m_CurrentTopoNode;

    double                                                                      m_VoxelSizeTabletop;
    double                                                                      m_VoxelSizeObservation;
    bool                                                                        m_bRegisterAndCorrectSweep;
    std::string                                                                 m_sRegisteredPoseLocation;

};

template <class PointType>
CloudMergeNode<PointType>::CloudMergeNode(ros::NodeHandle nh) : m_TransformListener(nh,ros::Duration(1000)), m_MongodbInterface(nh)
{
    ROS_INFO_STREAM("Cloud merge node initialized");

    m_NodeHandle = nh;
    m_CurrentTopoNode = "WayPoint1";
    m_RGB_it.reset(new image_transport::ImageTransport(nh));
    m_Depth_it.reset(new image_transport::ImageTransport(nh));

    m_MessageSynchronizer.reset( new Synchronizer(SyncPolicy(15), m_DepthSubscriber, m_RGBSubscriber, m_RGBCameraInfoSubscriber, m_DepthCameraInfoSubscriber) );
    m_MessageSynchronizer->registerCallback(boost::bind(&CloudMergeNode::imageCallback, this, _1, _2, _3, _4));

    m_SubscriberControl = m_NodeHandle.subscribe("/ptu/log",1, &CloudMergeNode::controlCallback,this);
    m_SubscriberTopoNode = m_NodeHandle.subscribe("/current_node",1, &CloudMergeNode::topoNodeCallback,this);

    m_PublisherIntermediateCloud = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/intermediate_point_cloud", 1);
    m_PublisherMergedCloud = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/merged_point_cloud", 1);
    m_PublisherMergedCloudDownsampled = m_NodeHandle.advertise<sensor_msgs::PointCloud2>("/local_metric_map/merged_point_cloud_downsampled", 1);

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
        m_SubscriberPointCloud = m_NodeHandle.subscribe(cloud_channel,1, &CloudMergeNode<PointType>::pointCloudCallback,this);

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
        found = m_NodeHandle.getParam("input_caminfo_rgb",caminfo_channel);
        if (!found)
        {
            caminfo_channel = "/head_xtion/rgb/camera_info";
        }
        m_RGBCameraInfoSubscriber.subscribe(m_NodeHandle, caminfo_channel,       1);
        ROS_INFO_STREAM("Subscribed to rgb camera info topic "<<caminfo_channel);

        std::string caminfo_channel_depth;
        found = m_NodeHandle.getParam("input_caminfo_depth",caminfo_channel_depth);
        if (!found)
        {
            caminfo_channel_depth = "/head_xtion/depth/camera_info";
        }
        m_DepthCameraInfoSubscriber.subscribe(m_NodeHandle, caminfo_channel_depth,       1);
        ROS_INFO_STREAM("Subscribed to depth camera info topic "<<caminfo_channel_depth);

    }

    m_bAquireData = false;
    m_bAquisitionPhase = false;

    scan_skip_counter = 0;

//    bool found = m_NodeHandle.getParam("/log_run_name",m_LogRunName);
//    if (!found)
//    {
//        ROS_INFO_STREAM("Parameter \"/log_run_name\" hasn't been defined. Defaulting to patrol_run_1");
//        m_LogRunName = "patrol_run_1";
//    }

    bool found = m_NodeHandle.getParam("save_intermediate_clouds",m_bSaveIntermediateData);
    if (!m_bSaveIntermediateData)
    {
        ROS_INFO_STREAM("Not saving intermediate point clouds.");
    } else {
        ROS_INFO_STREAM("Saving intermediate point clouds.");
    }

    bool doCleanup=true;
    m_NodeHandle.param<bool>("cleanup",doCleanup,true);
    m_NodeHandle.param<int>("max_instances",m_MaxInstances,-1);


    if (!doCleanup)
    {
        ROS_INFO_STREAM("Not removing old data at startup.");
    } else {
        ROS_INFO_STREAM("Will remove old data at startup.");
        SemanticMapSummaryParser summaryParser;
        summaryParser.removeSemanticMapData();
    }

    if (m_MaxInstances != -1)
    {
        ROS_INFO_STREAM("Maximum number of instances per observations is "<<m_MaxInstances);
        SemanticMapSummaryParser summaryParser;
        summaryParser.removeSemanticMapObservationInstances<PointType>(m_MaxInstances,m_bCacheOldData);
    } else {
        ROS_INFO_STREAM("Maximum number of instances hasn't been defined -> storing all the data.");
    }


    m_LogNameInitialized = false;
    m_LogName = "";
    m_SemanticRoomId = -1;

    m_NodeHandle.param<bool>("log_to_db",m_bLogToDB,true);
    if (m_bLogToDB)
    {
        ROS_INFO_STREAM("Logging intermediate point clouds to the database. This could take a lot of disk space.");
    } else {
        ROS_INFO_STREAM("NOT logging intermediate point clouds to the database.");
    }

    m_NodeHandle.param<bool>("cache_old_data",m_bCacheOldData,true);
    if (m_bCacheOldData)
    {
        ROS_INFO_STREAM("Old data will be cached. This could take a lot of disk space.");
    } else {
        ROS_INFO_STREAM("Old data will be deleted.");
    }

    m_NodeHandle.param<bool>("save_intermediate_images",m_bSaveIntermediateImages,false);
    if (m_bSaveIntermediateImages)
    {
        ROS_INFO_STREAM("Intermediate images will be saved. This could take a lot of disk space.");
    } else {
        ROS_INFO_STREAM("Intermediate images will NOT be saved.");
    }



    m_NodeHandle.param<double>("voxel_size_table_top",m_VoxelSizeTabletop,0.01);
    m_NodeHandle.param<double>("voxel_size_observation",m_VoxelSizeObservation,0.05);
    double cutoffDistance;
    m_NodeHandle.param<double>("point_cutoff_distance",cutoffDistance,4.0);
    m_CloudMerge.setMaximumPointDistance(cutoffDistance);

    ROS_INFO_STREAM("Voxel size for the table top point cloud is "<<m_VoxelSizeTabletop);
    ROS_INFO_STREAM("Voxel size for the observation and metaroom point cloud is "<<m_VoxelSizeObservation);
    ROS_INFO_STREAM("Point cutoff distance set to "<<cutoffDistance);


    m_NodeHandle.param<bool>("register_and_correct_sweep",m_bRegisterAndCorrectSweep,true);
    if (m_bRegisterAndCorrectSweep)
    {
         m_NodeHandle.param<std::string>("registered_poses_location",m_sRegisteredPoseLocation,"");
         if ((m_sRegisteredPoseLocation == "") || (m_sRegisteredPoseLocation == "default"))
         {
             // default path
             passwd* pw = getpwuid(getuid());
             std::string path(pw->pw_dir);
             path+="/.ros/semanticMap/registration_transforms.txt";
             if ( ! boost::filesystem::exists( path ) )
             {
                 ROS_INFO_STREAM("File containing precalibrated sweep positions cannot be found at "<<path);
                 m_bRegisterAndCorrectSweep = false;
             } else {
               m_sRegisteredPoseLocation = path;
             }
         } else {
             if ( ! boost::filesystem::exists( m_sRegisteredPoseLocation ) )
             {
                 ROS_INFO_STREAM("File containing precalibrated sweep positions cannot be found at "<<m_sRegisteredPoseLocation);
                 m_bRegisterAndCorrectSweep = false;
             }
         }

    } else {
        ROS_INFO_STREAM("Will not correct the intermediate point clouds of the sweeps using precalibrated sweep positions.");
    }

    if (m_bRegisterAndCorrectSweep)
    {
        ROS_INFO_STREAM("Precalibrated sweep positions will be loaded from "<<m_sRegisteredPoseLocation<<". The sweep intermediate point clouds will be corrected.");
    }

}
template <class PointType>
CloudMergeNode<PointType>::~CloudMergeNode()
{

}

template <class PointType>
void CloudMergeNode<PointType>::imageCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg, const sensor_msgs::CameraInfoConstPtr& info_msg_depth)
{

    if (!m_bUseImages)
    {
        return; // we don't need to process images
    }

    if (m_bAquireData && m_bAquisitionPhase)
    {
        scan_skip_counter++;
        if (scan_skip_counter < 5)
//            return;

        m_CloudMerge.addIntermediateImage(depth_msg, rgb_msg, info_msg, info_msg_depth);
    }

}



template <class PointType>
void CloudMergeNode<PointType>::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (m_bUseImages)
    {
        return; // we don't need to process images
    }

    //ROS_INFO_STREAM("Point cloud callback");
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
void CloudMergeNode<PointType>::topoNodeCallback(const std_msgs::String& topoNodeString)
{
    m_CurrentTopoNode = topoNodeString.data;
}

template <class PointType>
void CloudMergeNode<PointType>::controlCallback(const std_msgs::String& controlString)
{
//    ROS_INFO_STREAM("Received control string "<<controlString);

    if (controlString.data == "start_sweep")
    {
        ROS_INFO_STREAM("Pan tilt sweep started -> acquire data");
        m_bAquisitionPhase = true;

         // initialize room
         aSemanticRoom.setSaveIntermediateClouds(m_bSaveIntermediateData);

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

        // publish the merged cloud for table detection
        m_CloudMerge.subsampleMergedCloud(m_VoxelSizeTabletop,m_VoxelSizeTabletop,m_VoxelSizeTabletop);
        CloudPtr merged_cloud = m_CloudMerge.getMergedCloud();
        if (merged_cloud->points.size() != 0)
        { // only process this room if it has any points

            sensor_msgs::PointCloud2 msg_cloud;
            pcl::toROSMsg(*merged_cloud, msg_cloud);
            m_PublisherMergedCloud.publish(msg_cloud);


            CloudPtr merged_cloud = m_CloudMerge.getMergedCloud();
            // add complete cloud to semantic room
            aSemanticRoom.setCompleteRoomCloud(merged_cloud);

            // subsample again for visualization and metaroom purposes
            m_CloudMerge.subsampleMergedCloud(m_VoxelSizeObservation,m_VoxelSizeObservation,m_VoxelSizeObservation);
            merged_cloud = m_CloudMerge.getMergedCloud();
            pcl::toROSMsg(*merged_cloud, msg_cloud);
            m_PublisherMergedCloudDownsampled.publish(msg_cloud);

            // set room end time
            aSemanticRoom.setRoomLogEndTime(ros::Time::now().toBoost());

            // set room patrol number and room id
            int roomId, runNumber;
            findSemanticRoomIDAndLogName(aSemanticRoom, roomId, runNumber);
            m_SemanticRoomId = roomId;

            m_LogRunName = "patrol_run_";
            m_LogRunName += QString::number(runNumber).toStdString();

            static std::locale loc(std::cout.getloc(),
                                   new boost::posix_time::time_facet("%Y%m%d"));

            std::basic_stringstream<char> wss;
            wss.imbue(loc);
            wss << ros::Time::now().toBoost();
            m_LogName = wss.str()+"_"+m_LogRunName;
            ROS_INFO_STREAM("Log name set to "<<m_LogName);

            aSemanticRoom.setRoomRunNumber(m_SemanticRoomId);
            aSemanticRoom.setRoomLogName(m_LogName);
            if (m_CurrentTopoNode != "none")
            {
                aSemanticRoom.setRoomStringId(m_CurrentTopoNode);
                ROS_INFO_STREAM("Set room waypoint to "<<m_CurrentTopoNode);
            }


            SemanticRoomXMLParser<PointType> parser;
            ROS_INFO_STREAM("Saving semantic room file");
            std::string roomXMLPath = parser.saveRoomAsXML(aSemanticRoom);
            ROS_INFO_STREAM("Saved semantic room");

            if (m_bRegisterAndCorrectSweep)
            {
                // load precalibrated camera poses
                std::vector<tf::StampedTransform> regTransforms = semantic_map_registration_transforms::loadRegistrationTransforms("default", true);
                if (regTransforms.size() != aSemanticRoom.getIntermediateClouds().size())
                {
                    ROS_ERROR_STREAM("Cannot correct sweep as the number of precalibrated sweep poses is not the same as the number of intermediate point clouds in this sweep "/*<<regTransforms.size<<"  "<<aSemanticRoom.getIntermediateClouds().size()*/);
                } else {
                    // create corrected camera parameters
                    // TODO load this from file
                    ROS_INFO_STREAM("Reprojecting and registering sweep...");

                    image_geometry::PinholeCameraModel aCameraModel = semantic_map_registration_transforms::loadCameraParameters();
                    if (aCameraModel.fx() == -1)
                    {
                        // no cam model loaded. set defaults
                        ROS_INFO_STREAM("Could not find camera parameters file. Setting defaults.");
                        sensor_msgs::CameraInfo camInfo;
                        camInfo.P = {540.0, 0.0, 319.5, 0.0, 0.0, 540.0, 219.5, 0.0,0.0, 0.0, 1.0,0.0};
                        camInfo.D = {0,0,0,0,0};
                        aCameraModel.fromCameraInfo(camInfo);
                    }


                    auto origTransforms = aSemanticRoom.getIntermediateCloudTransforms();
                    tf::StampedTransform origin = origTransforms[0];
                    aSemanticRoom.clearIntermediateCloudRegisteredTransforms(); // just in case they had already been set
                    for (size_t i=0; i<origTransforms.size(); i++)
                    {
                        tf::StampedTransform transform = origTransforms[i];
                        transform.setOrigin(regTransforms[i].getOrigin());
                        transform.setBasis(regTransforms[i].getBasis());
                        aSemanticRoom.addIntermediateCloudCameraParametersCorrected(aCameraModel);
                        aSemanticRoom.addIntermediateRoomCloudRegisteredTransform(transform);
                    }
                    // reproject individual clouds
                    semantic_map_room_utilities::reprojectIntermediateCloudsUsingCorrectedParams<PointType>(aSemanticRoom);
                    // rebuild merged cloud
                    semantic_map_room_utilities::rebuildRegisteredCloud<PointType>(aSemanticRoom);
                    // transform merged cloud to map frame
                    CloudPtr completeCloud = aSemanticRoom.getCompleteRoomCloud();
                    pcl_ros::transformPointCloud(*completeCloud, *completeCloud,origin);
                    ROS_INFO_STREAM("..done");
                    aSemanticRoom.setCompleteRoomCloud(completeCloud);
                    std::string roomXMLPath = parser.saveRoomAsXML(aSemanticRoom);
                    unsigned found = roomXMLPath.find_last_of("/");
                    std::string base_path = roomXMLPath.substr(0,found+1);
                    RegistrationFeatures reg(true);
                    reg.saveOrbFeatures<PointType>(aSemanticRoom,base_path);

                }
            }

            // Pulbish room observation
            semantic_map::RoomObservation obs_msg;
            obs_msg.xml_file_name = roomXMLPath;

            // before publising, check whether some data needs to be removed
            if (m_MaxInstances != -1)
            {
                SemanticMapSummaryParser summaryParser;
                summaryParser.removeSemanticMapObservationInstances<PointType>(m_MaxInstances,m_bCacheOldData);
            }

            m_PublisherRoomObservation.publish(obs_msg);

            if (m_bLogToDB)
            {

                // log in separate thread
                /*auto future = */QtConcurrent::run(m_MongodbInterface, &MongodbInterface::logRoomToDB<PointType>,aSemanticRoom,roomXMLPath);
//                m_MongodbInterface.logRoomToDB(aSemanticRoom,roomXMLPath);
            }


        } else {
            ROS_INFO_STREAM("Observation point cloud is empty, discarding it. This shouldn't happen, it could be a problem with the camera driver or images.");
        }
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


        // get intermediate images (in case we want to save them)
        auto intermediate_rgb_images = m_CloudMerge.getIntermediateRGBImages();
        auto intermediate_depth_images = m_CloudMerge.getIntermediateDepthImages();

        // process the accumulated point cloud               

        CloudPtr transformed_cloud = m_CloudMerge.subsampleIntermediateCloud();
        if (transformed_cloud->points.size() == 0)
        {
            std::cout<<"Intermediate cloud has zero points. Skipping."<<std::endl;
            return;
        }


        try{
            if (transformed_cloud->points.size() > 0)
            {
                tf::StampedTransform transform;

                sensor_msgs::PointCloud2 temp_msg;
                pcl::toROSMsg(*transformed_cloud, temp_msg);
                temp_msg.header = pcl_conversions::fromPCL(transformed_cloud->header);
                m_PublisherIntermediateCloud.publish(temp_msg); // publish in the local frame of reference

                m_TransformListener.waitForTransform("/map", transformed_cloud->header.frame_id,temp_msg.header.stamp, ros::Duration(20.0) );
                m_TransformListener.lookupTransform("/map", transformed_cloud->header.frame_id,
                                                    temp_msg.header.stamp, transform);

                // add intermediate cloud (local frame of ref) and transform to semantic room
                int intCloudId;
                if (m_CloudMerge.m_IntermediateFilteredDepthCamInfo)
                {
                    image_geometry::PinholeCameraModel cloudCameraParams;
                    cloudCameraParams.fromCameraInfo(m_CloudMerge.m_IntermediateFilteredDepthCamInfo);
                    intCloudId = aSemanticRoom.addIntermediateRoomCloud(transformed_cloud, transform,cloudCameraParams);
                } else {
                    intCloudId = aSemanticRoom.addIntermediateRoomCloud(transformed_cloud, transform);
                }

                if (m_bSaveIntermediateImages)
                {

                    // find rgb and depth transforms
                    tf::StampedTransform rgb_transform, depth_transform;
                    std::string rgb_frame, depth_frame;
                    depth_frame = m_CloudMerge.m_IntermediateFilteredDepthCamInfo->header.frame_id;
                    rgb_frame = m_CloudMerge.m_IntermediateFilteredRGBCamInfo->header.frame_id;

                    m_TransformListener.waitForTransform("/map", rgb_frame,m_CloudMerge.m_IntermediateFilteredRGBCamInfo->header.stamp, ros::Duration(20.0) );
                    m_TransformListener.lookupTransform("/map", rgb_frame, m_CloudMerge.m_IntermediateFilteredRGBCamInfo->header.stamp, rgb_transform);

                    m_TransformListener.waitForTransform("/map", depth_frame,m_CloudMerge.m_IntermediateFilteredDepthCamInfo->header.stamp, ros::Duration(20.0) );
                    m_TransformListener.lookupTransform("/map", depth_frame, m_CloudMerge.m_IntermediateFilteredDepthCamInfo->header.stamp, depth_transform);


                    // camera parameters
                    image_geometry::PinholeCameraModel rgbCameraParams, depthCameraParams;
                    depthCameraParams.fromCameraInfo(m_CloudMerge.m_IntermediateFilteredDepthCamInfo);
                    rgbCameraParams.fromCameraInfo(m_CloudMerge.m_IntermediateFilteredRGBCamInfo);


                    aSemanticRoom.addIntermediateCloudImages(intermediate_rgb_images, intermediate_depth_images,
                                                             rgb_transform, depth_transform,
                                                             rgbCameraParams, depthCameraParams);
                }

                m_CloudMerge.transformIntermediateCloud(transform,"/map");
                transformed_cloud = m_CloudMerge.getIntermediateCloud();

                if (m_CloudMerge.m_IntermediateFilteredDepthImage)
                {
                    m_RosPublisherIntermediateDepth.publish(m_CloudMerge.m_IntermediateFilteredDepthImage);
                }
                if (m_CloudMerge.m_IntermediateFilteredRGBImage)
                {
                    m_RosPublisherIntermediateRGB.publish(m_CloudMerge.m_IntermediateFilteredRGBImage);
                }
                if (m_CloudMerge.m_IntermediateFilteredDepthCamInfo)
                {
                    m_RosPublisherIntermediateDepthCamInfo.publish(m_CloudMerge.m_IntermediateFilteredDepthCamInfo);
                }

                //            ROS_INFO_STREAM("Transformed cloud aquisition time "<<transformed_cloud->header.stamp<<"  and frame "<<transformed_cloud->header.frame_id<<"  and points "<<transformed_cloud->points.size());
            }

        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        m_CloudMerge.processIntermediateCloud();
    }

    if ((controlString.data == "preempted") && m_bAquisitionPhase)
    {
        ROS_INFO_STREAM("The pan tilt sweep was preempted while data was being collected. Abort current observation.");
        m_bAquireData = false;

        aSemanticRoom.clearIntermediateClouds();
        aSemanticRoom.resetRoomTransform();

        m_CloudMerge.resetIntermediateCloud();
        m_CloudMerge.resetMergedCloud();
    }
}

template <class PointType>
void CloudMergeNode<PointType>::findSemanticRoomIDAndLogName(SemanticRoom<PointType>& aSemanticRoom, int& roomRunNumber, int& roomLogName)
{
    SemanticMapSummaryParser summaryParser;
    summaryParser.createSummaryXML<PointType>();
    summaryParser.refresh();
    std::vector<Entities> allRooms = summaryParser.getRooms();

    if (m_bLogToDB)
    {
        std::vector<Entities> allMongoRooms = m_MongodbInterface.getMongodbRooms<PointType>();
        allRooms.insert(allRooms.end(), allMongoRooms.begin(), allMongoRooms.end());
    }

    int currentRoomID = -1;
    int highestRoomID = -1;
    int highestPatrolRunNumber = -1;
    int currentRoomPatrolNumber = -1;

    for (size_t i=0; i<allRooms.size(); i++)
    {
        QString roomXmlFile(allRooms[i].roomXmlFile.c_str());
        int roomId, patrolNumber;
        getRoomIDAndPatrolNumber(roomXmlFile, roomId, patrolNumber);
        if (patrolNumber>highestPatrolRunNumber)
        {
            highestPatrolRunNumber = patrolNumber;
        }

        ROS_DEBUG_STREAM("Room xml "<<roomXmlFile.toStdString()<<"  room ID "<<roomId<<"  patrol number "<<patrolNumber);
    }

    ROS_DEBUG_STREAM("Highest patrol run number "<<highestPatrolRunNumber);

    // search for a matchin observation in the latest patrol run.
    // if there is a matching observation, increment the patrol run
    // if there isn't, add this observation to the latest patrol run but increment


    if (highestPatrolRunNumber != -1)
    {
        for (size_t i=0; i<allRooms.size(); i++)
        {
            QString roomXmlFile(allRooms[i].roomXmlFile.c_str());
            int roomId, patrolNumber;
            getRoomIDAndPatrolNumber(roomXmlFile, roomId, patrolNumber);
            if (patrolNumber==highestPatrolRunNumber)
            {
                // check for a match using centroid distance
                double centroidDistance = pcl::distances::l2(aSemanticRoom.getCentroid(),allRooms[i].centroid);
                if ((centroidDistance < ROOM_CENTROID_DISTANCE) )
                {
                    // found a match
                    currentRoomPatrolNumber = highestPatrolRunNumber+1; // increment the patrol number
                    currentRoomID = 0; // first room in the next patrol run
                    ROS_DEBUG_STREAM("Found a matching room. "<<roomId);
                    break;
                } else {
                    if (roomId > highestRoomID)
                    {
                        highestRoomID = roomId;
                    }
                }
            }
        }
    } else {
        // no patrol runs performed so far. Start counting from 1
        highestPatrolRunNumber = 1;
    }

    if (currentRoomID == -1)
    {
        // couldn't find a match, add the room in the latest patrol run
        ROS_DEBUG_STREAM("Couldn't find a matching room. Incrementing highest room number");
        currentRoomPatrolNumber = highestPatrolRunNumber;
        currentRoomID = highestRoomID + 1;
    }

    roomRunNumber = currentRoomID;
    roomLogName = currentRoomPatrolNumber;
//    QString room_log_name = "patrol_run_"+QString::number(currentRoomPatrolNumber);

}
template <class PointType>

void CloudMergeNode<PointType>::getRoomIDAndPatrolNumber(QString roomXmlFile, int& roomId, int& patrolNumber)
{
    roomId = -1;
    patrolNumber = -1;

    int lastIndex = roomXmlFile.lastIndexOf("/");
    if (lastIndex == -1)
    {
        return;
    }

    QString roomFolderPath = roomXmlFile.left(lastIndex);
    int roomIDindex = roomFolderPath.lastIndexOf("_");
    if (roomIDindex == -1)
    {
        return;
    }

    roomId = roomFolderPath.right(roomFolderPath.length()-roomIDindex-1).toInt();
    int patrolFolderIndex = roomFolderPath.lastIndexOf("/");
    if (patrolFolderIndex == -1)
    {
        return;
    }

    QString patrolFolderPath = roomFolderPath.left(patrolFolderIndex);
    int patrolNumberIndex = patrolFolderPath.lastIndexOf("_");
    if (patrolNumberIndex == -1)
    {
        return;
    }

    patrolNumber = patrolFolderPath.right(patrolFolderPath.length()-patrolNumberIndex-1).toInt();
}


#endif
