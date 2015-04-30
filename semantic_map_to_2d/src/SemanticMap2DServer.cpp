#include <semantic_map_to_2d/SemanticMap2dServer.h>

using namespace octomap;
using octomap_msgs::Octomap;

SemanticMap2DServer::SemanticMap2DServer(ros::NodeHandle private_nh_)
    : m_nh(),
      m_octree(NULL),
      m_worldFrameId("/map"),
      m_res(0),
      m_treeDepth(0),
      m_maxTreeDepth(0),
      //m_probHit(0.7), m_probMiss(0.4),
      //m_thresMin(0.12), m_thresMax(0.97),
      m_occupancyMinZ(0.3),
      m_occupancyMaxZ(1.9),
      m_minSizeX(0.0), m_minSizeY(0.0),
      m_filterSpeckles(true)
{
    ros::NodeHandle private_nh(private_nh_);

    private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
    private_nh.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
    private_nh.param("min_x_size", m_minSizeX,m_minSizeX);
    private_nh.param("min_y_size", m_minSizeY,m_minSizeY);

    private_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);

    //private_nh.param("sensor_model/hit", m_probHit, m_probHit);
    //private_nh.param("sensor_model/miss", m_probMiss, m_probMiss);
    //private_nh.param("sensor_model/min", m_thresMin, m_thresMin);
    //private_nh.param("sensor_model/max", m_thresMax, m_thresMax);


    m_octree = new OcTree(m_res);

    m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("/waypoint_map", 5, 
                                                       true); // latched always
    m_changeWaypointService = m_nh.advertiseService("set_waypoint", &SemanticMap2DServer::changeWayPoint, this);

}

SemanticMap2DServer::~SemanticMap2DServer(){
    if (m_octree){
        delete m_octree;
        m_octree = NULL;
    }
}

bool SemanticMap2DServer::openFile(const std::string& filename){
    if (filename.length() <= 3)
        return false;

    std::string suffix = filename.substr(filename.length()-3, 3);
    if (suffix== ".bt"){
        if (!m_octree->readBinary(filename)){
            return false;
        }
    } else if (suffix == ".ot"){
        AbstractOcTree* tree = AbstractOcTree::read(filename);
        if (!tree){
            return false;
        }
        if (m_octree){
            delete m_octree;
            m_octree = NULL;
        }
        m_octree = dynamic_cast<OcTree*>(tree);
        if (!m_octree){
            ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
            return false;
        }

    } else{
        return false;
    }

    ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(),m_octree->size());

    m_treeDepth = m_octree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;
    m_res = m_octree->getResolution();
    m_gridmap.info.resolution = m_res;
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);

    m_updateBBXMin[0] = m_octree->coordToKey(minX);
    m_updateBBXMin[1] = m_octree->coordToKey(minY);
    m_updateBBXMin[2] = m_octree->coordToKey(minZ);

    m_updateBBXMax[0] = m_octree->coordToKey(maxX);
    m_updateBBXMax[1] = m_octree->coordToKey(maxY);
    m_updateBBXMax[2] = m_octree->coordToKey(maxZ);

    traverseOctomap();

    return true;

}


void SemanticMap2DServer::traverseOctomap(const ros::Time& rostime){

    // call pre-traversal hook:
    handlePreNodeTraversal(rostime);

    // now, traverse all leafs in the tree:
    for (OcTree::iterator it = m_octree->begin(m_maxTreeDepth),
         end = m_octree->end(); it != end; ++it)
    {
        bool inUpdateBBX = isInUpdateBBX(it);

        if (m_octree->isNodeOccupied(*it)){
            double z = it.getZ();
            if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
            {
                double x = it.getX();
                double y = it.getY();

                // Ignore speckles in the map:
                if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(it.getKey())){
                    ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
                    continue;
                } // else: current octree node is no speckle, send it out


                update2DMap(it, true);
                if (inUpdateBBX)
                    update2DMap(it, true);

            }
        } else{ // node not occupied => mark as free in 2D map if unknown so far
            double z = it.getZ();
            if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
            {
                update2DMap(it, false);

                if (inUpdateBBX)
                    update2DMap(it, false);

            }
        }
    }

    // call post-traversal hook:
    handlePostNodeTraversal(rostime);

}

bool SemanticMap2DServer::changeWayPoint(WaypointSrv::Request  &req,
                                   WaypointSrv::Response &res)
{
    ROS_INFO("Switching waypoint...");
    ROS_INFO("Requesting octomap from semantic map server....");
    ros::ServiceClient client = m_nh.serviceClient<semantic_map_publisher::ObservationOctomapService>
                                ("/semantic_map_publisher/SemanticMapPublisher/ObservationOctomapService");
    semantic_map_publisher::ObservationOctomapService srv;
    srv.request.waypoint_id = req.waypoint;
    srv.request.resolution=0.05;
    if (! client.call(srv))   {
      ROS_ERROR("Could not call semantic map to get a octomap!");
      return false;
    }
    
    AbstractOcTree* tree = octomap_msgs::fullMsgToMap(srv.response.octomap);
    if (!tree){
        ROS_ERROR("Failed to recreate octomap");
        return false;
    }
    if (m_octree){
        delete m_octree;
        m_octree = NULL;
    }
    m_octree = dynamic_cast<OcTree*>(tree);
    
    m_treeDepth = m_octree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;
    m_res = m_octree->getResolution();
    m_gridmap.info.resolution = m_res;
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);

    m_updateBBXMin[0] = m_octree->coordToKey(minX);
    m_updateBBXMin[1] = m_octree->coordToKey(minY);
    m_updateBBXMin[2] = m_octree->coordToKey(minZ);

    m_updateBBXMax[0] = m_octree->coordToKey(maxX);
    m_updateBBXMax[1] = m_octree->coordToKey(maxY);
    m_updateBBXMax[2] = m_octree->coordToKey(maxZ);

    traverseOctomap();

    //res.map.header.frame_id = m_worldFrameId;
    //res.map.header.stamp = ros::Time::now();
    res.is_ok = true;
    //res.response ="Ok.";

    return true;
}


void SemanticMap2DServer::handlePreNodeTraversal(const ros::Time& rostime){
    // init projected 2D map:
    m_gridmap.header.frame_id = m_worldFrameId;
    m_gridmap.header.stamp = rostime;

    // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octree->getMetricMin(minX, minY, minZ);
    m_octree->getMetricMax(maxX, maxY, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey = m_octree->coordToKey(minPt, m_maxTreeDepth);
    octomap::OcTreeKey maxKey = m_octree->coordToKey(maxPt, m_maxTreeDepth);
    
    ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

    // add padding if requested (= new min/maxPts in x&y):
    double halfPaddedX = 0.5*m_minSizeX;
    double halfPaddedY = 0.5*m_minSizeY;
    minX = std::min(minX, -halfPaddedX);
    maxX = std::max(maxX, halfPaddedX);
    minY = std::min(minY, -halfPaddedY);
    maxY = std::max(maxY, halfPaddedY);
    minPt = octomap::point3d(minX, minY, minZ);
    maxPt = octomap::point3d(maxX, maxY, maxZ);

    OcTreeKey paddedMaxKey;
    if (!m_octree->coordToKeyChecked(minPt, m_maxTreeDepth, m_paddedMinKey)){
        ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
        return;
    }
    if (!m_octree->coordToKeyChecked(maxPt, m_maxTreeDepth, paddedMaxKey)){
        ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
        return;
    }

    ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
    assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

    m_multires2DScale = 1 << (m_treeDepth - m_maxTreeDepth);
    m_gridmap.info.width = (paddedMaxKey[0] - m_paddedMinKey[0])/m_multires2DScale +1;
    m_gridmap.info.height = (paddedMaxKey[1] - m_paddedMinKey[1])/m_multires2DScale +1;

    int mapOriginX = minKey[0] - m_paddedMinKey[0];
    int mapOriginY = minKey[1] - m_paddedMinKey[1];
    assert(mapOriginX >= 0 && mapOriginY >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin = m_octree->keyToCoord(m_paddedMinKey, m_treeDepth);
    double gridRes = m_octree->getNodeSize(m_maxTreeDepth);
    m_projectCompleteMap = true;
    m_gridmap.info.resolution = gridRes;
    m_gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
    m_gridmap.info.origin.position.y = origin.y() - gridRes*0.5;

    if (m_maxTreeDepth != m_treeDepth) {
        m_gridmap.info.origin.position.x -= m_res/2.0;
        m_gridmap.info.origin.position.y -= m_res/2.0;
    }
    

    // Initialise a blank unknown (-1) map.
    m_gridmap.data.clear();
    m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);

}

void SemanticMap2DServer::handlePostNodeTraversal(const ros::Time& rostime){
    m_mapPub.publish(m_gridmap);
}


void SemanticMap2DServer::update2DMap(const OcTreeT::iterator& it, bool occupied){

    // update 2D map (occupied always overrides):

    if (it.getDepth() == m_maxTreeDepth){
        unsigned idx = mapIdx(it.getKey());
        if (occupied)
            m_gridmap.data[idx] = 100;
        else if (m_gridmap.data[idx] == -1){
            m_gridmap.data[idx] = 0;
        }

    } else{
        int intSize = 1 << (m_maxTreeDepth - it.getDepth());
        octomap::OcTreeKey minKey=it.getIndexKey();
        for(int dx=0; dx < intSize; dx++){
            int i = (minKey[0]+dx - m_paddedMinKey[0])/m_multires2DScale;
            for(int dy=0; dy < intSize; dy++){
                unsigned idx = mapIdx(i, (minKey[1]+dy - m_paddedMinKey[1])/m_multires2DScale);
                if (occupied)
                    m_gridmap.data[idx] = 100;
                else if (m_gridmap.data[idx] == -1){
                    m_gridmap.data[idx] = 0;
                }
            }
        }
    }


}



bool SemanticMap2DServer::isSpeckleNode(const OcTreeKey&nKey) const {
    OcTreeKey key;
    bool neighborFound = false;
    for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
        for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
            for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
                if (key != nKey){
                    OcTreeNode* node = m_octree->search(key);
                    if (node && m_octree->isNodeOccupied(node)){
                        // we have a neighbor => break!
                        neighborFound = true;
                    }
                }
            }
        }
    }

    return neighborFound;
}





