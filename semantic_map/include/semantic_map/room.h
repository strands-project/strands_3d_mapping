#ifndef __SEMANTIC_ROOM__H
#define __SEMANTIC_ROOM__H

#include <stdio.h>
#include <iosfwd>
#include <stdlib.h>
#include <string>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>

#include <image_geometry/pinhole_camera_model.h>

#include <ros/time.h>
#include <tf/tf.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "roombase.h"



template <class PointType>
class SemanticRoom : public RoomBase<PointType>{
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    struct IntermediatePositionImages
    {
        std::vector<cv::Mat>                vIntermediateDepthImages;
        std::vector<cv::Mat>                vIntermediateRGBImages;
        tf::StampedTransform                intermediateDepthTransform;
        tf::StampedTransform                intermediateRGBTransform;
        image_geometry::PinholeCameraModel  intermediateRGBCamParams;
        image_geometry::PinholeCameraModel  intermediateDepthCamParams;
        int                                 numRGBImages, numDepthImages;
        bool                                images_loaded;
    };

private:

    CloudPtr                                         m_DynamicClustersCloud;
    bool                                             m_DynamicClustersLoaded;
    std::string                                      m_DynamicClustersFilename;
    // intermediate room clouds
    std::vector<CloudPtr>                            m_vIntermediateRoomClouds;     
    std::vector<tf::StampedTransform>                m_vIntermediateRoomCloudTransforms;
    std::vector<tf::StampedTransform>                m_vIntermediateRoomCloudTransformsRegistered;
    std::vector<bool>                                m_vIntermediateRoomCloudsLoaded;
    std::vector<std::string>                         m_vIntermediateRoomCloudsFilenames;
    std::vector<image_geometry::PinholeCameraModel>  m_vIntermediateRoomCloudsCamParams;
    std::vector<image_geometry::PinholeCameraModel>  m_vIntermediateRoomCloudsCamParamsCorrected;

    // intermediate cloud images
    std::vector<IntermediatePositionImages>                m_vIntermediatePositionImages;

    std::string                                      m_RoomStringId;
    int                                              m_RoomRunNumber;
    std::string                                      m_RoomLogName;
    boost::posix_time::ptime                         m_RoomLogStartTime, m_RoomLogEndTime;

    bool                                             m_bIsMetaRoom;
    bool                                             m_bSaveIntermediateClouds;

public:

    SemanticRoom(bool saveIntermediateClouds=true);
    ~SemanticRoom();

    auto getDynamicClustersCloud() -> decltype(m_DynamicClustersCloud);
    void setDynamicClustersCloud(CloudPtr dynCl);
    void setDynamicClustersCloud(std::string dynClF);
    bool getDynamicClustersCloudLoaded();

    void clearIntermediateClouds();
    int addIntermediateRoomCloud(CloudPtr intermediateCloud, tf::StampedTransform cloud_tf, image_geometry::PinholeCameraModel cloudCamParams=image_geometry::PinholeCameraModel());
    void addIntermediateCloudImages(std::vector<sensor_msgs::Image::ConstPtr> sensor_rgb_images, std::vector<sensor_msgs::Image::ConstPtr> sensor_depth_images,
                                    tf::StampedTransform rgb_transform, tf::StampedTransform depth_transform,
                                    image_geometry::PinholeCameraModel rgb_params, image_geometry::PinholeCameraModel depth_params);

    void addIntermediateCloudImages(IntermediatePositionImages newImages);
    auto getIntermdiatePositionImages()  -> decltype(m_vIntermediatePositionImages);

    int addIntermediateRoomCloud(std::string filename, tf::StampedTransform cloud_tf, image_geometry::PinholeCameraModel cloudCamParams=image_geometry::PinholeCameraModel());
    bool getSaveIntermediateClouds();
    void setSaveIntermediateClouds(bool saveIntermediate);
    auto getIntermediateClouds() -> decltype (m_vIntermediateRoomClouds);

    void clearIntermediateCloudRegisteredTransforms();
    void addIntermediateRoomCloudRegisteredTransform(tf::StampedTransform cloud_reg_tf);
    std::vector<tf::StampedTransform> getIntermediateCloudTransformsRegistered();
    std::vector<tf::StampedTransform> getIntermediateCloudTransforms();
    std::vector<image_geometry::PinholeCameraModel> getIntermediateCloudCameraParameters();
    std::vector<image_geometry::PinholeCameraModel> getIntermediateCloudCameraParametersCorrected();
    void clearIntermediateCloudCameraParametersCorrected();
    void addIntermediateCloudCameraParametersCorrected(image_geometry::PinholeCameraModel params);
    std::vector<bool>   getIntermediateCloudsLoaded();
    std::vector<std::string>   getIntermediateCloudsFilenames();

    void setRoomRunNumber(const int& roomRunNumber);
    int getRoomRunNumber();
    void setRoomStringId(const std::string& roomId);
    std::string getRoomStringId();
    void setRoomLogName(const std::string& roomLogName);
    std::string getRoomLogName();
    void setRoomLogStartTime(const boost::posix_time::ptime& logStartTime);
    boost::posix_time::ptime getRoomLogStartTime();
    void setRoomLogEndTime(const boost::posix_time::ptime& logEndTime);
    boost::posix_time::ptime getRoomLogEndTime();
    boost::posix_time::ptime getRoomTime();

    bool operator==(const SemanticRoom& rhs); // equality operator -> deep comparison of all fields

    static Eigen::Vector4f computeCentroid(CloudPtr cloud);

    // public members (should be private ...)
    int                                              pan_start, pan_step, pan_end;
    int                                              tilt_start, tilt_step, tilt_end;

};

#include "room.hpp"

#endif
