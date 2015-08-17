#include "semantic_map/roombase.h"

template <class PointType>
RoomBase<PointType>::RoomBase() : m_CompleteRoomCloud(new Cloud()), m_RoomCentroid(0.0,0.0,0.0,0.0), m_CompleteRoomCloudFilename(""), m_CompleteRoomCloudLoaded(false),
    m_InteriorRoomCloud(new Cloud()), m_InteriorRoomCloudLoaded(false), m_DeNoisedRoomCloud(new Cloud()), m_DeNoisedRoomCloudLoaded(false)
{
    m_RoomTransform = Eigen::Matrix4f::Identity();
}

template <class PointType>
RoomBase<PointType>::~RoomBase()
{
}

template <class PointType>
void RoomBase<PointType>::setCompleteRoomCloud(CloudPtr completeCloud)
{
    *m_CompleteRoomCloud = *completeCloud;
    m_CompleteRoomCloud->header = completeCloud->header;
    m_CompleteRoomCloudLoaded = true;

    // compute and set the centroid
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*completeCloud, centroid);
    this->setCentroid(centroid);
}

template <class PointType>
void RoomBase<PointType>::setCompleteRoomCloud(std::string completeCloud)
{
    m_CompleteRoomCloudLoaded = false;
    m_CompleteRoomCloudFilename = completeCloud;
}

template <class PointType>
typename pcl::PointCloud<PointType>::Ptr RoomBase<PointType>::getCompleteRoomCloud()
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

template <class PointType>
bool RoomBase<PointType>::getCompleteRoomCloudLoaded()
{
    return m_CompleteRoomCloudLoaded;
}

template <class PointType>
std::string RoomBase<PointType>::getCompleteRoomCloudFilename()
{
    return m_CompleteRoomCloudFilename;
}

template <class PointType>
void RoomBase<PointType>::setInteriorRoomCloud(CloudPtr interiorCloud)
{
    *m_InteriorRoomCloud = *interiorCloud;
    m_InteriorRoomCloudLoaded = true;
}

template <class PointType>
void RoomBase<PointType>::setInteriorRoomCloud(std::string interiorCloud)
{
    m_InteriorRoomCloudLoaded = false;
    m_InteriorRoomCloudFilename = interiorCloud;
}

template <class PointType>
typename pcl::PointCloud<PointType>::Ptr RoomBase<PointType>::getInteriorRoomCloud()
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

template <class PointType>
bool RoomBase<PointType>::getInteriorRoomCloudLoaded()
{
    return m_InteriorRoomCloudLoaded;
}

template <class PointType>
std::string RoomBase<PointType>::getInteriorRoomCloudFilename()
{
    return m_InteriorRoomCloudFilename;
}

template <class PointType>
void RoomBase<PointType>::setDeNoisedRoomCloud(CloudPtr denoisedCloud)
{
    *m_DeNoisedRoomCloud = *denoisedCloud;
    m_DeNoisedRoomCloudLoaded = true;
}

template <class PointType>
void RoomBase<PointType>::setDeNoisedRoomCloud(std::string denoisedCloud)
{
    m_DeNoisedRoomCloudLoaded = false;
    m_DeNoisedRoomCloudFilename = denoisedCloud;
}

template <class PointType>
typename pcl::PointCloud<PointType>::Ptr RoomBase<PointType>::getDeNoisedRoomCloud()
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

template <class PointType>
bool RoomBase<PointType>::getDeNoisedRoomCloudLoaded()
{
    return m_DeNoisedRoomCloudLoaded;
}

template <class PointType>
std::string RoomBase<PointType>::getDeNoisedRoomCloudFilename()
{
    return m_DeNoisedRoomCloudFilename;
}

template <class PointType>
void RoomBase<PointType>::setWallsCloud(CloudPtr wallsCloud)
{
    *m_WallsCloud = *wallsCloud;
    m_WallsCloudLoaded = true;
}

template <class PointType>
void RoomBase<PointType>::setWallsCloud(std::string wallsCloud)
{
    m_WallsCloudLoaded = false;
    m_WallsCloudFilename = wallsCloud;
}

template <class PointType>
typename pcl::PointCloud<PointType>::Ptr RoomBase<PointType>::getWallsCloud()
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

template <class PointType>
bool RoomBase<PointType>::getWallsCloudLoaded()
{
    return m_WallsCloudLoaded;
}

template <class PointType>
std::string RoomBase<PointType>::getWallsCloudFilename()
{
    return m_WallsCloudFilename;
}

template <class PointType>
void RoomBase<PointType>::setCentroid(Eigen::Vector4f centroid)
{
    m_RoomCentroid = centroid;
}

template <class PointType>
Eigen::Vector4f RoomBase<PointType>::getCentroid()
{
    return m_RoomCentroid;
}

template <class PointType>
Eigen::Matrix4f RoomBase<PointType>::getRoomTransform()
{
    return m_RoomTransform;
}

template <class PointType>
void RoomBase<PointType>::setRoomTransform(Eigen::Matrix4f transform)
{
    m_RoomTransform = transform;
}

template <class PointType>
void RoomBase<PointType>::resetRoomTransform()
{
    m_RoomTransform = Eigen::Matrix4f::Identity();
}
