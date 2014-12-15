#include "semantic_map/metaroom_update_iteration.h"

template<class PointType>
MetaRoomUpdateIteration<PointType>::MetaRoomUpdateIteration() : differenceMetaRoomToRoom(new Cloud()), differenceRoomToMetaRoom(new Cloud()),
    clustersToBeAdded(new Cloud()), clustersToBeRemoved(new Cloud()), metaRoomInteriorCloud(new Cloud())
{
    differenceMetaRoomToRoomLoaded = false;
    differenceRoomToMetaRoomLoaded = false;
    clustersToBeAddedLoaded = false;
    clustersToBeRemovedLoaded = false;
    metaRoomInteriorCloudLoaded = false;
    roomRunNumber = -1;
    roomLogName = "";
    differenceMetaRoomToRoomFilename = "";
    differenceRoomToMetaRoomFilename = "";
    clustersToBeAddedFilename = "";
    clustersToBeRemovedFilename = "";
    metaRoomInteriorCloudFilename = "";
}

template<class PointType>
typename pcl::PointCloud<PointType>::Ptr MetaRoomUpdateIteration<PointType>::getDifferenceMetaRoomToRoom()
{
    if (!differenceMetaRoomToRoomLoaded)
    {
        // first load the complete point cloud
        std::cout<<"Loading differenceMetaRoomToRoom "<<differenceMetaRoomToRoomFilename<<std::endl;
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read (differenceMetaRoomToRoomFilename, *cloud);
        *differenceMetaRoomToRoom = *cloud;
        differenceMetaRoomToRoomLoaded = true;
    }
    return differenceMetaRoomToRoom;
}

template<class PointType>
typename pcl::PointCloud<PointType>::Ptr MetaRoomUpdateIteration<PointType>::getDifferenceRoomToMetaRoom()
{
    if (!differenceRoomToMetaRoomLoaded)
    {
        // first load the complete point cloud
        std::cout<<"Loading differenceMetaRoomToRoom "<<differenceRoomToMetaRoomFilename<<std::endl;
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read (differenceRoomToMetaRoomFilename, *cloud);
        *differenceRoomToMetaRoom = *cloud;
        differenceRoomToMetaRoomLoaded = true;
    }
    return differenceRoomToMetaRoom;
}

template<class PointType>
typename pcl::PointCloud<PointType>::Ptr         MetaRoomUpdateIteration<PointType>::getClustersToBeAdded()
{
    if (!clustersToBeAddedLoaded)
    {
        // first load the clusters to be added
        std::cout<<"Loading clusters to be added "<<clustersToBeAddedFilename<<std::endl;
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read (clustersToBeAddedFilename, *cloud);
        *clustersToBeAdded = *cloud;
        clustersToBeAddedLoaded = true;
    }
    return clustersToBeAdded;
}

template<class PointType>
typename pcl::PointCloud<PointType>::Ptr         MetaRoomUpdateIteration<PointType>::getClustersToBeRemoved()
{
    if (!clustersToBeRemovedLoaded)
    {
        // first load the clusters to be removed
        std::cout<<"Loading clusters to be removed "<<clustersToBeRemovedFilename<<std::endl;
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read (clustersToBeRemovedFilename, *cloud);
        *clustersToBeRemoved = *cloud;
        clustersToBeRemovedLoaded = true;
    }
    return clustersToBeRemoved;
}

template<class PointType>
typename pcl::PointCloud<PointType>::Ptr MetaRoomUpdateIteration<PointType>::getMetaRoomInteriorCloud()
{
    if (!metaRoomInteriorCloudLoaded)
    {
        // first load the metaroom interior cloud
        std::cout<<"Loading the metaroom interior cloud "<<metaRoomInteriorCloudFilename<<std::endl;
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read (metaRoomInteriorCloudFilename, *cloud);
        *metaRoomInteriorCloud = *cloud;
        metaRoomInteriorCloudLoaded = true;
    }
    return metaRoomInteriorCloud;
}

template<class PointType>
void MetaRoomUpdateIteration<PointType>::setDifferenceMetaRoomToRoom(CloudPtr diffMRToR)
{
    *differenceMetaRoomToRoom = *diffMRToR;
    differenceMetaRoomToRoomLoaded = true;
}

template<class PointType>
void MetaRoomUpdateIteration<PointType>::setDifferenceMetaRoomToRoom(std::string diffMRToRFile)
{
    differenceMetaRoomToRoomFilename = diffMRToRFile;
    differenceMetaRoomToRoomLoaded = false;
}

template<class PointType>
void MetaRoomUpdateIteration<PointType>::setDifferenceRoomToMetaRoom(CloudPtr diffRToMR)
{
    *differenceRoomToMetaRoom = *diffRToMR;
    differenceRoomToMetaRoomLoaded = true;
}

template<class PointType>
void MetaRoomUpdateIteration<PointType>::setDifferenceRoomToMetaRoom(std::string diffRToMRFile)
{
    differenceRoomToMetaRoomFilename = diffRToMRFile;
    differenceRoomToMetaRoomLoaded = false;
}

template<class PointType>
void MetaRoomUpdateIteration<PointType>::setClustersToBeAdded(CloudPtr clusters)
{
    *clustersToBeAdded = *clusters;
    clustersToBeAddedLoaded = true;
}

template<class PointType>
void MetaRoomUpdateIteration<PointType>::setClustersToBeAdded(std::string clusters)
{
    clustersToBeAddedFilename = clusters;
    clustersToBeAddedLoaded = false;
}

template<class PointType>
void MetaRoomUpdateIteration<PointType>::setClustersToBeRemoved(CloudPtr clusters)
{
    *clustersToBeRemoved = *clusters;
    clustersToBeRemovedLoaded = true;
}

template<class PointType>
void MetaRoomUpdateIteration<PointType>::setClustersToBeRemoved(std::string clusters)
{
    clustersToBeRemovedFilename = clusters;
    clustersToBeRemovedLoaded = false;
}

template<class PointType>
void MetaRoomUpdateIteration<PointType>::setMetaRoomInteriorCloud(CloudPtr interior)
{
    *metaRoomInteriorCloud = *interior;
    metaRoomInteriorCloudLoaded = true;
}

template<class PointType>
void MetaRoomUpdateIteration<PointType>::setMetaRoomInteriorCloud(std::string interior)
{
    metaRoomInteriorCloudFilename = interior;
    metaRoomInteriorCloudLoaded = false;
}
