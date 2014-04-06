#ifndef __ROOM_BASE__H
#define __ROOM_BASE__H

#include <stdio.h>
#include <iostream>
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

    RoomBase() : m_CompleteRoomCloud(new Cloud()), m_RoomCentroid(0.0,0.0,0.0,0.0), m_CompleteRoomCloudFilename(""), m_CompleteRoomCloudLoaded(false),
        m_InteriorRoomCloud(new Cloud()), m_InteriorRoomCloudLoaded(false), m_DeNoisedRoomCloud(new Cloud()), m_DeNoisedRoomCloudLoaded(false)
    {
        m_RoomTransform = Eigen::Matrix4f::Identity();
    }

    ~RoomBase()
    {
    }

    void setCompleteRoomCloud(CloudPtr completeCloud)
    {
        *m_CompleteRoomCloud = *completeCloud;
        m_CompleteRoomCloudLoaded = true;

        // compute and set the centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*completeCloud, centroid);
        this->setCentroid(centroid);
    }

    void setCompleteRoomCloud(std::string completeCloud)
    {
        m_CompleteRoomCloudLoaded = false;
        m_CompleteRoomCloudFilename = completeCloud;
    }

    CloudPtr getCompleteRoomCloud()
    {
        if (!m_CompleteRoomCloudLoaded)
        {
            // first load the complete point cloud
            std::cout<<"Loading complete room cloud "<<m_CompleteRoomCloudFilename<<std::endl;
            pcl::PCDReader reader;
            CloudPtr cloud (new Cloud);
            reader.read (m_CompleteRoomCloudFilename, *cloud);
            this->setCompleteRoomCloud(cloud);

            // compute and set the centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud, centroid);
            this->setCentroid(centroid);
        }

        return m_CompleteRoomCloud;
    }

    bool getCompleteRoomCloudLoaded()
    {
        return m_CompleteRoomCloudLoaded;
    }

    std::string getCompleteRoomCloudFilename()
    {
        return m_CompleteRoomCloudFilename;
    }


    void setInteriorRoomCloud(CloudPtr interiorCloud)
    {
        *m_InteriorRoomCloud = *interiorCloud;
        m_InteriorRoomCloudLoaded = true;
    }

    void setInteriorRoomCloud(std::string interiorCloud)
    {
        m_InteriorRoomCloudLoaded = false;
        m_InteriorRoomCloudFilename = interiorCloud;
    }

    CloudPtr getInteriorRoomCloud()
    {
        if (!m_InteriorRoomCloudLoaded)
        {
            // first load the complete point cloud
            std::cout<<"Loading interior room cloud "<<m_InteriorRoomCloudFilename<<std::endl;
            pcl::PCDReader reader;
            CloudPtr cloud (new Cloud);
            reader.read (m_InteriorRoomCloudFilename, *cloud);
            this->setInteriorRoomCloud(cloud);
        }

        return m_InteriorRoomCloud;
    }

    bool getInteriorRoomCloudLoaded()
    {
        return m_InteriorRoomCloudLoaded;
    }

    std::string getInteriorRoomCloudFilename()
    {
        return m_InteriorRoomCloudFilename;
    }


        void setDeNoisedRoomCloud(CloudPtr denoisedCloud)
    {
        *m_DeNoisedRoomCloud = *denoisedCloud;
        m_DeNoisedRoomCloudLoaded = true;
    }

    void setDeNoisedRoomCloud(std::string denoisedCloud)
    {
        m_DeNoisedRoomCloudLoaded = false;
        m_DeNoisedRoomCloudFilename = denoisedCloud;
    }

    CloudPtr getDeNoisedRoomCloud()
    {
        if (!m_DeNoisedRoomCloudLoaded)
        {
            // first load the complete point cloud
            std::cout<<"Loading DeNoised room cloud "<<m_DeNoisedRoomCloudFilename<<std::endl;
            pcl::PCDReader reader;
            CloudPtr cloud (new Cloud);
            reader.read (m_DeNoisedRoomCloudFilename, *cloud);
            this->setDeNoisedRoomCloud(cloud);
        }

        return m_DeNoisedRoomCloud;
    }

    bool getDeNoisedRoomCloudLoaded()
    {
        return m_DeNoisedRoomCloudLoaded;
    }

    std::string getDeNoisedRoomCloudFilename()
    {
        return m_DeNoisedRoomCloudFilename;
    }

    //

    void setWallsCloud(CloudPtr wallsCloud)
    {
        *m_WallsCloud = *wallsCloud;
        m_WallsCloudLoaded = true;
    }

    void setWallsCloud(std::string wallsCloud)
    {
        m_WallsCloudLoaded = false;
        m_WallsCloudFilename = wallsCloud;
    }

    CloudPtr getWallsCloud()
    {
        if (!m_WallsCloudLoaded)
        {
            // first load the wals point cloud
            std::cout<<"Loading walls room cloud "<<m_WallsCloudFilename<<std::endl;
            pcl::PCDReader reader;
            CloudPtr cloud (new Cloud);
            reader.read (m_WallsCloudFilename, *cloud);
            this->setWallsCloud(cloud);
        }

        return m_WallsCloud;
    }

    bool getWallsCloudLoaded()
    {
        return m_WallsCloudLoaded;
    }

    std::string getWallsCloudFilename()
    {
        return m_WallsCloudFilename;
    }

    void setCentroid(Eigen::Vector4f centroid)
    {
        m_RoomCentroid = centroid;
    }

    Eigen::Vector4f getCentroid()
    {
        return m_RoomCentroid;
    }

    Eigen::Matrix4f getRoomTransform()
    {
        return m_RoomTransform;
    }

    void setRoomTransform(Eigen::Matrix4f transform)
    {
        m_RoomTransform = transform;
    }

    void resetRoomTransform()
    {
        m_RoomTransform = Eigen::Matrix4f::Identity();
    }




};


#endif
