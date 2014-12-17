#ifndef __ROOM_BASE__H
#define __ROOM_BASE__H

#include <stdio.h>
#include <iosfwd>
#include <stdlib.h>
#include <string>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>

#include "ros/time.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "tf/tf.h"

template <class PointType>
class RoomBase {
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

protected:

    CloudPtr                                         m_CompleteRoomCloud;
    bool                                             m_CompleteRoomCloudLoaded;
    std::string                                      m_CompleteRoomCloudFilename;

    CloudPtr                                         m_InteriorRoomCloud;
    bool                                             m_InteriorRoomCloudLoaded;
    std::string                                      m_InteriorRoomCloudFilename;

    CloudPtr                                         m_DeNoisedRoomCloud;
    bool                                             m_DeNoisedRoomCloudLoaded;
    std::string                                      m_DeNoisedRoomCloudFilename;

    CloudPtr                                         m_WallsCloud;
    bool                                             m_WallsCloudLoaded;
    std::string                                      m_WallsCloudFilename;

    Eigen::Vector4f                                  m_RoomCentroid;
    Eigen::Matrix4f                                  m_RoomTransform;

public:

    RoomBase();
    ~RoomBase();

    void setCompleteRoomCloud(CloudPtr completeCloud);
    void setCompleteRoomCloud(std::string completeCloud);
    CloudPtr getCompleteRoomCloud();
    bool getCompleteRoomCloudLoaded();
    std::string getCompleteRoomCloudFilename();

    void setInteriorRoomCloud(CloudPtr interiorCloud);
    void setInteriorRoomCloud(std::string interiorCloud);
    CloudPtr getInteriorRoomCloud();
    bool getInteriorRoomCloudLoaded();
    std::string getInteriorRoomCloudFilename();

    void setDeNoisedRoomCloud(CloudPtr denoisedCloud);
    void setDeNoisedRoomCloud(std::string denoisedCloud);
    CloudPtr getDeNoisedRoomCloud();
    bool getDeNoisedRoomCloudLoaded();
    std::string getDeNoisedRoomCloudFilename();

    void setWallsCloud(CloudPtr wallsCloud);
    void setWallsCloud(std::string wallsCloud);
    CloudPtr getWallsCloud();
    bool getWallsCloudLoaded();
    std::string getWallsCloudFilename();

    void setCentroid(Eigen::Vector4f centroid);
    Eigen::Vector4f getCentroid();

    Eigen::Matrix4f getRoomTransform();
    void setRoomTransform(Eigen::Matrix4f transform);
    void resetRoomTransform();

};

#include "roombase.hpp"

#endif
