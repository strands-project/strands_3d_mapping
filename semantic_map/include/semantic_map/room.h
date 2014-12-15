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
    std::vector<bool>                                m_vIntermediateRoomCloudsLoaded;
    std::vector<std::string>                         m_vIntermediateRoomCloudsFilenames;
    std::vector<image_geometry::PinholeCameraModel>  m_vIntermediateRoomCloudsCamParams;

    // intermediate cloud images
    std::vector<IntermediatePositionImages>                m_vIntermediatePositionImages;

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

        IntermediatePositionImages intermediate_images;
        intermediate_images.vIntermediateDepthImages.assign(depth_images.begin(), depth_images.end());
        intermediate_images.vIntermediateRGBImages.assign(rgb_images.begin(), rgb_images.end());
        intermediate_images.intermediateRGBCamParams = rgb_params;
        intermediate_images.intermediateDepthCamParams = depth_params;
        intermediate_images.intermediateRGBTransform = rgb_transform;
        intermediate_images.intermediateDepthTransform = depth_transform;
        intermediate_images.numRGBImages = rgb_images.size();
        intermediate_images.numDepthImages = depth_images.size();

        m_vIntermediatePositionImages.push_back(intermediate_images);
    }

    void addIntermediateCloudImages(IntermediatePositionImages newImages)
    {
        m_vIntermediatePositionImages.push_back(newImages);
    }

    auto getIntermdiatePositionImages()  -> decltype(m_vIntermediatePositionImages)
    {
        return m_vIntermediatePositionImages;
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
