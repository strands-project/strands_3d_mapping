#ifndef __META_ROOM_UPDATE_ITERATION__H
#define __META_ROOM_UPDATE_ITERATION__H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

template <class PointType>
class MetaRoomUpdateIteration{
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;


    MetaRoomUpdateIteration();

    CloudPtr        getDifferenceMetaRoomToRoom();
    CloudPtr        getDifferenceRoomToMetaRoom();
    CloudPtr        getClustersToBeAdded();
    CloudPtr        getClustersToBeRemoved();
    CloudPtr        getMetaRoomInteriorCloud();

    void setDifferenceMetaRoomToRoom(CloudPtr diffMRToR);
    void setDifferenceMetaRoomToRoom(std::string diffMRToRFile);
    void setDifferenceRoomToMetaRoom(CloudPtr diffRToMR);
    void setDifferenceRoomToMetaRoom(std::string diffRToMRFile);
    void setClustersToBeAdded(CloudPtr clusters);
    void setClustersToBeAdded(std::string clusters);
    void setClustersToBeRemoved(CloudPtr clusters);
    void setClustersToBeRemoved(std::string clusters);
    void setMetaRoomInteriorCloud(CloudPtr interior);
    void setMetaRoomInteriorCloud(std::string interior);

    int                                              roomRunNumber;
    std::string                                      roomLogName;
    CloudPtr                                         differenceMetaRoomToRoom;
    std::string                                      differenceMetaRoomToRoomFilename;
    bool                                             differenceMetaRoomToRoomLoaded;

    CloudPtr                                         differenceRoomToMetaRoom;
    std::string                                      differenceRoomToMetaRoomFilename;
    bool                                             differenceRoomToMetaRoomLoaded;

    CloudPtr                                         clustersToBeAdded;
    std::string                                      clustersToBeAddedFilename;
    bool                                             clustersToBeAddedLoaded;

    CloudPtr                                         clustersToBeRemoved;
    std::string                                      clustersToBeRemovedFilename;
    bool                                             clustersToBeRemovedLoaded;

    CloudPtr                                         metaRoomInteriorCloud;
    std::string                                      metaRoomInteriorCloudFilename;
    bool                                             metaRoomInteriorCloudLoaded;
};

#include "metaroom_update_iteration.hpp"

#endif
