#ifndef __META_ROOM__H
#define __META_ROOM__H

#include <stdio.h>
#include <iosfwd>
#include <stdlib.h>
#include <string>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/distances.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <vector>

#include "ros/time.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include "tf/tf.h"

#include "ndt_registration.h"
#include "roombase.h"
#include "room.h"
#include "constants.h"
#include "room_xml_parser.h"
#include "occlusion_checker.h"
#include "metaroom_update_iteration.h"

template <class PointType>
class MetaRoom : public RoomBase<PointType> {
public:

    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef pcl::search::KdTree<PointType> Tree;

private:

    std::vector<MetaRoomUpdateIteration<PointType>>                     m_MetaRoomUpdateIterations;
    pcl::ModelCoefficients::Ptr                                         m_MetaRoomCeilingPrimitive;
    bool                                                                m_MetaRoomCeilingPrimitiveDirection;
    pcl::ModelCoefficients::Ptr                                         m_MetaRoomFloorPrimitive;
    bool                                                                m_MetaRoomFloorPrimitiveDirection;
    std::vector<pcl::ModelCoefficients::Ptr>                            m_vMetaRoomWallPrimitives;
    std::vector<bool>                                                   m_vMetaRoomWallPrimitivesDirections;
    tf::Vector3                                                         m_SensorOrigin;

    CloudPtr                                                            m_ConsistencyUpdateCloud;
    std::string                                                         m_ConsistencyUpdateCloudFilename;
    bool                                                                m_ConsistencyUpdateCloudLoaded;
    bool                                                                m_bSaveIntermediateSteps;
    bool                                                                m_bUpdateMetaroom;

public:


    MetaRoom(bool saveIntermediateSteps=true);
    ~MetaRoom();

    void setUpdateMetaroom(bool updateMetaroom);
    void resetMetaRoom();

    bool getSaveIntermediateSteps();
    void setSaveIntermediateSteps(bool saveSteps);

    std::pair<pcl::ModelCoefficients::Ptr,bool> getCeilingPrimitive();;
    void setCeilingPrimitive(pcl::ModelCoefficients::Ptr primitive,bool direction);
    std::pair<pcl::ModelCoefficients::Ptr,bool> getFloorPrimitive();
    void setFloorPrimitive(pcl::ModelCoefficients::Ptr primitive,bool direction);
    std::pair<std::vector<pcl::ModelCoefficients::Ptr>,std::vector<bool> > getWallPrimitives();
    void setWallPrimitives(std::vector<pcl::ModelCoefficients::Ptr> primitives ,std::vector<bool> directions);
    std::pair<std::vector<pcl::ModelCoefficients::Ptr>,std::vector<bool> > getBoundingPrimitives();

    std::vector<MetaRoomUpdateIteration<PointType>> getUpdateIterations();
    void addUpdateIteration(MetaRoomUpdateIteration<PointType> newIteration);

    void setConsistencyUpdateCloud(CloudPtr cons);
    void setConsistencyUpdateCloud(std::string cons);
    bool getConsistencyUpdateCloudLoaded();
    std::string getConsistencyUpdateCloudFilename();
    CloudPtr getConsistencyUpdateCloud();

    tf::Vector3 getSensorOrigin();
    void setSensorOrigin(tf::Vector3 so);

    bool    updateMetaRoom(SemanticRoom<PointType>& aRoom, std::string savePath="");

    void filterClustersBasedOnDistance(std::vector<CloudPtr>& clusters, double maxDistance);
    static std::vector<CloudPtr> clusterPointCloud(CloudPtr input_cloud, double tolerance = 0.05, int min_cluster_size = 100, int max_cluster_size=100000);
    static CloudPtr downsampleCloud(CloudPtr input, double leafSize = 0.01f);

};

#include "metaroom.hpp"

#endif
