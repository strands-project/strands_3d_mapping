
#ifndef OCTOMAP_TO_MAP_H
#define OCTOMAP_TO_MAP_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <octomap_msgs/Octomap.h>
#include <semantic_map_to_2d/ChangeWaypoint.h>
#include <octomap_msgs/conversions.h>
#include <semantic_map_publisher/ObservationOctomapService.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>


class SemanticMap2DServer{

public:
  typedef semantic_map_to_2d::ChangeWaypoint WaypointSrv;
  typedef octomap::OcTree OcTreeT;

  SemanticMap2DServer(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
  virtual ~SemanticMap2DServer();
  virtual bool changeWayPoint(WaypointSrv::Request  &req, WaypointSrv::Response &res);

  virtual bool openFile(const std::string& filename);

protected:
  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min){
    for (unsigned i=0; i<3; ++i)
      min[i] = std::min(in[i], min[i]);
  };
  
  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max){
    for (unsigned i=0; i<3; ++i)
      max[i] = std::max(in[i], max[i]);
  };
 
  /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const octomap::OcTree::iterator& it) const{
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
    return (key[0]+voxelWidth >= m_updateBBXMin[0]
         && key[1]+voxelWidth >= m_updateBBXMin[1]
         && key[0] <= m_updateBBXMax[0]
         && key[1] <= m_updateBBXMax[1]);
  }

  void traverseOctomap(const ros::Time& rostime = ros::Time::now());


  /**
  * @brief Find speckle nodes (single occupied voxels with no neighbors). 
           Only works on lowest resolution!
  * @param key
  * @return
  */
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  /// hook that is called before traversing all nodes
  virtual void handlePreNodeTraversal(const ros::Time& rostime);

  /// hook that is called after traversing all nodes
  virtual void handlePostNodeTraversal(const ros::Time& rostime);

  /// updates the downprojected 2D map as either occupied or free
  virtual void update2DMap(const OcTreeT::iterator& it, bool occupied);

  inline unsigned mapIdx(int i, int j) const{
    return m_gridmap.info.width*j + i;
  }

  inline unsigned mapIdx(const octomap::OcTreeKey& key) const{
    return mapIdx((key[0] - m_paddedMinKey[0])/m_multires2DScale,
        (key[1] - m_paddedMinKey[1])/m_multires2DScale);

  }

  /**
   * Adjust data of map due to a change in its info properties (origin or size,
   * resolution needs to stay fixed). map already contains the new map info,
   * but the data is stored according to oldMapInfo.
   */

  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& oldMapInfo) const;

  inline bool mapChanged(const nav_msgs::MapMetaData& oldMapInfo, const nav_msgs::MapMetaData& newMapInfo){
    return (    oldMapInfo.height != newMapInfo.height
             || oldMapInfo.width !=newMapInfo.width
             || oldMapInfo.origin.position.x != newMapInfo.origin.position.x
             || oldMapInfo.origin.position.y != newMapInfo.origin.position.y);
  }

  ros::NodeHandle m_nh;
  ros::Publisher  m_markerPub, m_binaryMapPub, m_fullMapPub, m_pointCloudPub, m_collisionObjectPub, m_mapPub, m_cmapPub, m_fmapPub, m_fmarkerPub;
  ros::ServiceServer m_changeWaypointService;

  octomap::OcTree* m_octree;
  octomap::KeyRay m_keyRay;  // temp storage for ray casting
  octomap::OcTreeKey m_updateBBXMin;
  octomap::OcTreeKey m_updateBBXMax;

  std::string m_worldFrameId; // the map frame
  std::string m_baseFrameId; // base of the robot for ground plane filtering
  bool m_useHeightMap;
  double m_colorFactor;

  bool m_latchedTopics;
  bool m_publishFreeSpace;

  double m_res;
  unsigned m_treeDepth;
  unsigned m_maxTreeDepth;
  double m_probHit;
  double m_probMiss;
  double m_thresMin;
  double m_thresMax;

  double m_occupancyMinZ;
  double m_occupancyMaxZ;
  double m_minSizeX;
  double m_minSizeY;
  bool m_filterSpeckles;

  bool m_filterGroundPlane;
  double m_groundFilterDistance;
  double m_groundFilterAngle;
  double m_groundFilterPlaneDistance;


  // downprojected 2D map:
  nav_msgs::OccupancyGrid m_gridmap;
  bool m_publish2DMap;
  bool m_mapOriginChanged;
  octomap::OcTreeKey m_paddedMinKey;
  unsigned m_multires2DScale;
  bool m_projectCompleteMap;
};

#endif
