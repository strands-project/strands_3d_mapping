#ifndef __SEMANTIC_ROOM__H
#define __SEMANTIC_ROOM__H

#include <stdio.h>
#include <iostream>
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

private:

    CloudPtr                                         m_DynamicClustersCloud;
    bool                                             m_DynamicClustersLoaded;
    std::string                                      m_DynamicClustersFilename;
    // intermediate room clouds
    std::vector<CloudPtr>                            m_vIntermediateRoomClouds;     
    std::vector<tf::StampedTransform>                m_vIntermediateRoomCloudTransforms;
    std::vector<bool>                                m_vIntermediateRoomCloudsLoaded;
    std::vector<std::string>                         m_vIntermediateRoomCloudsFilenames;
    std::vector<image_geometry::PinholeCameraModel>  m_vIntermediateRoomCloudsCamParams;

    // intermediate cloud images
    std::vector<std::vector<cv::Mat>>                m_vIntermediateDepthImages;
    std::vector<std::vector<cv::Mat>>                m_vIntermediateRGBImages;
    std::vector<tf::StampedTransform>                m_vIntermediateDepthTransforms;
    std::vector<tf::StampedTransform>                m_vIntermediateRGBTransforms;
    std::vector<image_geometry::PinholeCameraModel>  m_vIntermediateRGBCamParams;
    std::vector<image_geometry::PinholeCameraModel>  m_vIntermediateDepthCamParams;


    std::string                                      m_RoomStringId;
    int                                              m_RoomRunNumber;
    std::string                                      m_RoomLogName;
    boost::posix_time::ptime                         m_RoomLogStartTime, m_RoomLogEndTime;

    bool                                             m_bIsMetaRoom;
    bool                                             m_bSaveIntermediateClouds;

public:

    SemanticRoom(bool saveIntermediateClouds=true) : RoomBase<PointType>(), m_bSaveIntermediateClouds(saveIntermediateClouds), m_DynamicClustersCloud(new Cloud()),
        m_DynamicClustersLoaded(false), m_DynamicClustersFilename("")
    {
        m_RoomStringId = "";
        m_RoomLogName = "";
        m_RoomRunNumber = -1;
        m_bIsMetaRoom = false;
    }

    ~SemanticRoom()
    {

    }

    CloudPtr getDynamicClustersCloud()
    {
        if (!m_DynamicClustersLoaded)
        {
            // first load the complete point cloud
            std::cout<<"Loading dynamic clusters cloud "<<m_DynamicClustersFilename<<std::endl;
            pcl::PCDReader reader;
            CloudPtr cloud (new Cloud);
            if (m_DynamicClustersFilename!="")
            {
                reader.read (m_DynamicClustersFilename, *cloud);
            }
            this->setDynamicClustersCloud(cloud);
        }

        return m_DynamicClustersCloud;
    }

    void setDynamicClustersCloud(CloudPtr dynCl)
    {
        *m_DynamicClustersCloud = *dynCl;
        m_DynamicClustersLoaded = true;
    }

    void setDynamicClustersCloud(std::string dynClF)
    {
        m_DynamicClustersFilename = dynClF;
        m_DynamicClustersLoaded = false;
    }

    bool getDynamicClustersCloudLoaded()
    {
        return m_DynamicClustersLoaded;
    }

    void clearIntermediateClouds()
    {
        m_vIntermediateRoomClouds.clear();
        m_vIntermediateRoomCloudTransforms.clear();
    }

    int addIntermediateRoomCloud(CloudPtr intermediateCloud, tf::StampedTransform cloud_tf, image_geometry::PinholeCameraModel cloudCamParams=image_geometry::PinholeCameraModel())
    {
        CloudPtr newCloud(new Cloud);
        *newCloud = *intermediateCloud;
        m_vIntermediateRoomClouds.push_back(newCloud);
        m_vIntermediateRoomCloudTransforms.push_back(cloud_tf);
        m_vIntermediateRoomCloudsLoaded.push_back(true);
        m_vIntermediateRoomCloudsCamParams.push_back(cloudCamParams);

        return m_vIntermediateRoomClouds.size();
    }

    void addIntermediateCloudImages(std::vector<sensor_msgs::Image::ConstPtr> sensor_rgb_images, std::vector<sensor_msgs::Image::ConstPtr> sensor_depth_images,
                                    tf::StampedTransform rgb_transform, tf::StampedTransform depth_transform,
                                    image_geometry::PinholeCameraModel rgb_params, image_geometry::PinholeCameraModel depth_params)
    {
        std::vector<cv::Mat> rgb_images, depth_images;

        for_each( sensor_rgb_images.begin(), sensor_rgb_images.end(), [&] (sensor_msgs::Image::ConstPtr image)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(*image, "bgr8");
                rgb_images.push_back(cv_ptr->image);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        } );

        for_each( sensor_depth_images.begin(), sensor_depth_images.end(), [&] (decltype (*sensor_depth_images.begin()) image)
        {
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                cv_ptr = cv_bridge::toCvCopy(*image, "16UC1");
                depth_images.push_back(cv_ptr->image);
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        } );

        m_vIntermediateDepthImages.push_back(depth_images);
        m_vIntermediateRGBImages.push_back(rgb_images);

        m_vIntermediateDepthTransforms.push_back(depth_transform);
        m_vIntermediateRGBTransforms.push_back(rgb_transform);

        m_vIntermediateDepthCamParams.push_back(depth_params);
        m_vIntermediateRGBCamParams.push_back(rgb_params);
    }

    auto getIntermediateDepthImages() -> decltype(m_vIntermediateDepthImages)
    {
        return m_vIntermediateDepthImages;
    }

    auto getIntermediateRGBImages() -> decltype(m_vIntermediateRGBImages)
    {
        return m_vIntermediateRGBImages;
    }

    int addIntermediateRoomCloud(std::string filename, tf::StampedTransform cloud_tf, image_geometry::PinholeCameraModel cloudCamParams=image_geometry::PinholeCameraModel())
    {
        m_vIntermediateRoomCloudTransforms.push_back(cloud_tf);
        m_vIntermediateRoomCloudsLoaded.push_back(false);
        m_vIntermediateRoomCloudsFilenames.push_back(filename);
        m_vIntermediateRoomCloudsCamParams.push_back(cloudCamParams);

        return m_vIntermediateRoomCloudsFilenames.size();
    }

    bool getSaveIntermediateClouds()
    {
        return m_bSaveIntermediateClouds;
    }

    void setSaveIntermediateClouds(bool saveIntermediate)
    {
        m_bSaveIntermediateClouds = saveIntermediate;
    }

    std::vector<CloudPtr> getIntermediateClouds()
    {
        return m_vIntermediateRoomClouds;
    }

    std::vector<tf::StampedTransform> getIntermediateCloudTransforms()
    {
        return m_vIntermediateRoomCloudTransforms;
    }

    std::vector<image_geometry::PinholeCameraModel> getIntermediateCloudCameraParameters()
    {
        return m_vIntermediateRoomCloudsCamParams;
    }

    std::vector<bool>   getIntermediateCloudsLoaded()
    {
        return m_vIntermediateRoomCloudsLoaded;
    }

    std::vector<std::string>   getIntermediateCloudsFilenames()
    {
        return m_vIntermediateRoomCloudsFilenames;
    }

    void setRoomRunNumber(const int& roomRunNumber)
    {
        m_RoomRunNumber = roomRunNumber;
    }

    int getRoomRunNumber()
    {
        return m_RoomRunNumber;
    }

    void setRoomStringId(const std::string& roomId)
    {
        m_RoomStringId = roomId;
    }

    std::string getRoomStringId()
    {
        return m_RoomStringId;
    }

    void setRoomLogName(const std::string& roomLogName)
    {
        m_RoomLogName = roomLogName;
    }

    std::string getRoomLogName()
    {
        return m_RoomLogName;
    }

    void setRoomLogStartTime(const boost::posix_time::ptime& logStartTime)
    {
        m_RoomLogStartTime = logStartTime;
    }

    boost::posix_time::ptime getRoomLogStartTime()
    {
        return m_RoomLogStartTime;
    }

    void setRoomLogEndTime(const boost::posix_time::ptime& logEndTime)
    {
        m_RoomLogEndTime = logEndTime;
    }

    boost::posix_time::ptime getRoomLogEndTime()
    {
        return m_RoomLogEndTime;
    }

    boost::posix_time::ptime getRoomTime()
    {
        return m_RoomLogStartTime;
    }    

    bool operator==(const SemanticRoom& rhs) // equality operator -> deep comparison of all fields
    {
        if (m_RoomStringId != rhs.m_RoomStringId)
        {
            std::cout<<"Room string ID not equal"<<std::endl;
            return false;
        }

        if (m_RoomRunNumber != rhs.m_RoomRunNumber)
        {
            std::cout<<"Room run number not equal"<<std::endl;
            return false;
        }

        if (m_RoomLogName != rhs.m_RoomLogName)
        {
            std::cout<<"Room log name not equal"<<std::endl;
            return false;
        }

        if (m_RoomLogStartTime != rhs.m_RoomLogStartTime)
        {
            std::cout<<"Room log start time not equal"<<std::endl;
            return false;
        }

        if (m_RoomLogEndTime != rhs.m_RoomLogEndTime)
        {
            std::cout<<"Room log end time not equal"<<std::endl;
            return false;
        }

        if (this->m_CompleteRoomCloudLoaded != rhs.m_CompleteRoomCloudLoaded)
        {
            return false;
        }

        if (this->m_CompleteRoomCloudFilename != rhs.m_CompleteRoomCloudFilename)
        {
            return false;
        }

        if (this->m_CompleteRoomCloudLoaded)
        {
            if (this->m_CompleteRoomCloud->points.size() != rhs.m_CompleteRoomCloud->points.size())
            {
                return false;
            }
        }

        if (m_vIntermediateRoomClouds.size() != rhs.m_vIntermediateRoomClouds.size())
        {
            std::cout<<"Room intermediate cloud vector sizes not equal"<<std::endl;
            return false;
        }

        if (m_vIntermediateRoomCloudTransforms.size() != rhs.m_vIntermediateRoomCloudTransforms.size())
        {
            std::cout<<"Room intermediate cloud tranform vector sizes not equal"<<std::endl;
            return false;
        }

        if (m_vIntermediateRoomCloudsLoaded.size() != rhs.m_vIntermediateRoomCloudsLoaded.size())
        {
            return false;
        }

        if (m_vIntermediateRoomCloudsFilenames.size() != rhs.m_vIntermediateRoomCloudsFilenames.size())
        {
            return false;
        }

        for (size_t i=0; i<m_vIntermediateRoomClouds.size(); i++)
        {
            if (m_vIntermediateRoomCloudsLoaded[i] != rhs.m_vIntermediateRoomCloudsLoaded[i])
            {
                return false;
            }

            if (m_vIntermediateRoomCloudsFilenames[i] != rhs.m_vIntermediateRoomCloudsFilenames[i])
            {
                return false;
            }

            if (m_vIntermediateRoomCloudsLoaded[i])
            {
                if (m_vIntermediateRoomClouds[i]->points.size() != rhs.m_vIntermediateRoomClouds[i]->points.size())
                {
                    return false;
                }
            }
//            if (m_vIntermediateRoomCloudTransforms[i] != rhs.m_vIntermediateRoomCloudTransforms[i])
//            {
//                return false;
//            }
        }

        return true;
    }

    static Eigen::Vector4f computeCentroid(CloudPtr cloud)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        return centroid;
    }


};


#endif //SEMANTIC_ROOM
