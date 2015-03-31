#include <iostream>

#include "semantic_map/room.h"


template <class PointType>
SemanticRoom<PointType>::SemanticRoom(bool saveIntermediateClouds) : RoomBase<PointType>(), m_bSaveIntermediateClouds(saveIntermediateClouds), m_DynamicClustersCloud(new SemanticRoom::Cloud()),
    m_DynamicClustersLoaded(false), m_DynamicClustersFilename("")
{
    m_RoomStringId = "";
    m_RoomLogName = "";
    m_RoomRunNumber = -1;
    m_bIsMetaRoom = false;

    // initialize sweep parameters with defaults
    pan_start = -160; pan_step = 20; pan_end = 160; // 17 horizontal steps
    tilt_start = -30; tilt_step = 30; tilt_end = 30; // 3 vertical steps
}

template <class PointType>
SemanticRoom<PointType>::~SemanticRoom()
{

}

template <class PointType>
auto SemanticRoom<PointType>::getDynamicClustersCloud() -> decltype (m_DynamicClustersCloud)
{
    if (!m_DynamicClustersLoaded)
    {
        // first load the complete point cloud
        std::cout<<"Loading dynamic clusters cloud "<<m_DynamicClustersFilename<<std::endl;
        pcl::PCDReader reader;
        SemanticRoom<PointType>::CloudPtr cloud (new SemanticRoom<PointType>::Cloud);
        if (m_DynamicClustersFilename!="")
        {
            reader.read (m_DynamicClustersFilename, *cloud);
        }
        this->setDynamicClustersCloud(cloud);
    }

    return m_DynamicClustersCloud;
}

template <class PointType>
void SemanticRoom<PointType>::setDynamicClustersCloud(SemanticRoom<PointType>::CloudPtr dynCl)
{
    *m_DynamicClustersCloud = *dynCl;
    m_DynamicClustersLoaded = true;
}

template <class PointType>
void SemanticRoom<PointType>::setDynamicClustersCloud(std::string dynClF)
{
    m_DynamicClustersFilename = dynClF;
    m_DynamicClustersLoaded = false;
}

template <class PointType>
bool SemanticRoom<PointType>::getDynamicClustersCloudLoaded()
{
    return m_DynamicClustersLoaded;
}

template <class PointType>
void SemanticRoom<PointType>::clearIntermediateClouds()
{
    m_vIntermediateRoomClouds.clear();
    m_vIntermediateRoomCloudTransforms.clear();
}

template <class PointType>
int SemanticRoom<PointType>::addIntermediateRoomCloud(SemanticRoom<PointType>::CloudPtr intermediateCloud, tf::StampedTransform cloud_tf, image_geometry::PinholeCameraModel cloudCamParams)
{
    SemanticRoom<PointType>::CloudPtr newCloud(new SemanticRoom<PointType>::Cloud);
    *newCloud = *intermediateCloud;
    m_vIntermediateRoomClouds.push_back(newCloud);
    m_vIntermediateRoomCloudTransforms.push_back(cloud_tf);
    m_vIntermediateRoomCloudsLoaded.push_back(true);
    m_vIntermediateRoomCloudsCamParams.push_back(cloudCamParams);

    return m_vIntermediateRoomClouds.size();
}

template <class PointType>
void SemanticRoom<PointType>::addIntermediateCloudImages(std::vector<sensor_msgs::Image::ConstPtr> sensor_rgb_images, std::vector<sensor_msgs::Image::ConstPtr> sensor_depth_images,
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

    SemanticRoom<PointType>::IntermediatePositionImages intermediate_images;
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

template <class PointType>
void SemanticRoom<PointType>::addIntermediateCloudImages(SemanticRoom<PointType>::IntermediatePositionImages newImages)
{
    m_vIntermediatePositionImages.push_back(newImages);
}

template <class PointType>
auto SemanticRoom<PointType>::getIntermdiatePositionImages()  -> decltype(m_vIntermediatePositionImages)
{
    return m_vIntermediatePositionImages;
}

template <class PointType>
int SemanticRoom<PointType>::addIntermediateRoomCloud(std::string filename, tf::StampedTransform cloud_tf, image_geometry::PinholeCameraModel cloudCamParams)
{
    m_vIntermediateRoomCloudTransforms.push_back(cloud_tf);
    m_vIntermediateRoomCloudsLoaded.push_back(false);
    m_vIntermediateRoomCloudsFilenames.push_back(filename);
    m_vIntermediateRoomCloudsCamParams.push_back(cloudCamParams);

    return m_vIntermediateRoomCloudsFilenames.size();
}

template <class PointType>
bool SemanticRoom<PointType>::getSaveIntermediateClouds()
{
    return m_bSaveIntermediateClouds;
}

template <class PointType>
void SemanticRoom<PointType>::setSaveIntermediateClouds(bool saveIntermediate)
{
    m_bSaveIntermediateClouds = saveIntermediate;
}

template <class PointType>
auto SemanticRoom<PointType>::getIntermediateClouds() -> decltype(m_vIntermediateRoomClouds)
{
    return m_vIntermediateRoomClouds;
}

template <class PointType>
void SemanticRoom<PointType>::addIntermediateRoomCloudRegisteredTransform(tf::StampedTransform cloud_reg_tf)
{
    m_vIntermediateRoomCloudTransformsRegistered.push_back(cloud_reg_tf);
}

template <class PointType>
void SemanticRoom<PointType>::clearIntermediateCloudRegisteredTransforms()
{
    m_vIntermediateRoomCloudTransformsRegistered.clear();
}

template <class PointType>
std::vector<tf::StampedTransform> SemanticRoom<PointType>::getIntermediateCloudTransformsRegistered()
{
    return m_vIntermediateRoomCloudTransformsRegistered;
}

template <class PointType>
std::vector<tf::StampedTransform> SemanticRoom<PointType>::getIntermediateCloudTransforms()
{
    return m_vIntermediateRoomCloudTransforms;
}

template <class PointType>
std::vector<image_geometry::PinholeCameraModel> SemanticRoom<PointType>::getIntermediateCloudCameraParameters()
{
    return m_vIntermediateRoomCloudsCamParams;
}

template <class PointType>
std::vector<image_geometry::PinholeCameraModel> SemanticRoom<PointType>::getIntermediateCloudCameraParametersCorrected()
{
    return m_vIntermediateRoomCloudsCamParamsCorrected;
}

template <class PointType>
void SemanticRoom<PointType>::addIntermediateCloudCameraParametersCorrected(image_geometry::PinholeCameraModel params)
{
    m_vIntermediateRoomCloudsCamParamsCorrected.push_back(params);
}

template <class PointType>
void SemanticRoom<PointType>::clearIntermediateCloudCameraParametersCorrected()
{
   m_vIntermediateRoomCloudsCamParamsCorrected.clear();
}


template <class PointType>
std::vector<bool>   SemanticRoom<PointType>::getIntermediateCloudsLoaded()
{
    return m_vIntermediateRoomCloudsLoaded;
}

template <class PointType>
std::vector<std::string>   SemanticRoom<PointType>::getIntermediateCloudsFilenames()
{
    return m_vIntermediateRoomCloudsFilenames;
}

template <class PointType>
void SemanticRoom<PointType>::setRoomRunNumber(const int& roomRunNumber)
{
    m_RoomRunNumber = roomRunNumber;
}

template <class PointType>
int SemanticRoom<PointType>::getRoomRunNumber()
{
    return m_RoomRunNumber;
}

template <class PointType>
void SemanticRoom<PointType>::setRoomStringId(const std::string& roomId)
{
    m_RoomStringId = roomId;
}

template <class PointType>
std::string SemanticRoom<PointType>::getRoomStringId()
{
    return m_RoomStringId;
}

template <class PointType>
void SemanticRoom<PointType>::setRoomLogName(const std::string& roomLogName)
{
    m_RoomLogName = roomLogName;
}

template <class PointType>
std::string SemanticRoom<PointType>::getRoomLogName()
{
    return m_RoomLogName;
}

template <class PointType>
void SemanticRoom<PointType>::setRoomLogStartTime(const boost::posix_time::ptime& logStartTime)
{
    m_RoomLogStartTime = logStartTime;
}

template <class PointType>
boost::posix_time::ptime SemanticRoom<PointType>::getRoomLogStartTime()
{
    return m_RoomLogStartTime;
}

template <class PointType>
void SemanticRoom<PointType>::setRoomLogEndTime(const boost::posix_time::ptime& logEndTime)
{
    m_RoomLogEndTime = logEndTime;
}

template <class PointType>
boost::posix_time::ptime SemanticRoom<PointType>::getRoomLogEndTime()
{
    return m_RoomLogEndTime;
}

template <class PointType>
boost::posix_time::ptime SemanticRoom<PointType>::getRoomTime()
{
    return m_RoomLogStartTime;
}

template <class PointType>
bool SemanticRoom<PointType>::operator==(const SemanticRoom<PointType>& rhs) // equality operator -> deep comparison of all fields
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

    if (m_vIntermediateRoomCloudTransforms.size() != rhs.m_vIntermediateRoomCloudTransforms.size())
    {
        std::cout<<"Room intermediate cloud tranform vector sizes not equal"<<std::endl;
        return false;
    }

    if (m_vIntermediateRoomCloudTransformsRegistered.size() != rhs.m_vIntermediateRoomCloudTransformsRegistered.size())
    {
        std::cout<<"Room intermediate cloud registered tranform vector sizes not equal"<<std::endl;
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

template <class PointType>
Eigen::Vector4f SemanticRoom<PointType>::computeCentroid(SemanticRoom<PointType>::CloudPtr cloud)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    return centroid;
}
