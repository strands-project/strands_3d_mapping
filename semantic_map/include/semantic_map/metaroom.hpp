#include "semantic_map/metaroom.h"

template <class PointType>
MetaRoom<PointType>::MetaRoom(bool saveIntermediateSteps) : RoomBase<PointType>(), m_SensorOrigin(0.0,0.0,0.0), m_ConsistencyUpdateCloud(new Cloud()), m_bSaveIntermediateSteps(saveIntermediateSteps),
    m_bUpdateMetaroom(true)
{
    m_MetaRoomCeilingPrimitive = pcl::ModelCoefficients::Ptr(new (pcl::ModelCoefficients));
    m_MetaRoomCeilingPrimitive->values = std::vector<float>(4,0.0); // initialize with 0
    m_MetaRoomCeilingPrimitiveDirection = false;
    m_MetaRoomFloorPrimitive = pcl::ModelCoefficients::Ptr(new (pcl::ModelCoefficients));
    m_MetaRoomFloorPrimitive->values = std::vector<float>(4,0.0);
    m_MetaRoomFloorPrimitiveDirection = false;
    m_ConsistencyUpdateCloudLoaded = false;
    m_ConsistencyUpdateCloudFilename = "";
}

template <class PointType>
MetaRoom<PointType>::~MetaRoom()
{

}

template <class PointType>
void MetaRoom<PointType>::setUpdateMetaroom(bool updateMetaroom)
{
    m_bUpdateMetaroom = updateMetaroom;
}

template <class PointType>
void MetaRoom<PointType>::resetMetaRoom()
{
    this->m_CompleteRoomCloud = CloudPtr(new Cloud());
    m_MetaRoomUpdateIterations.clear();
    this->m_RoomCentroid = Eigen::Vector4f::Identity();
    this->m_CompleteRoomCloudLoaded = false;
    this->m_CompleteRoomCloudFilename = "";

    m_MetaRoomCeilingPrimitive = pcl::ModelCoefficients::Ptr(new (pcl::ModelCoefficients));
    m_MetaRoomFloorPrimitive = pcl::ModelCoefficients::Ptr(new (pcl::ModelCoefficients));
    m_vMetaRoomWallPrimitives.clear();
    m_MetaRoomUpdateIterations.clear();
}

template <class PointType>
bool MetaRoom<PointType>::getSaveIntermediateSteps()
{
    return m_bSaveIntermediateSteps;
}

template <class PointType>
void MetaRoom<PointType>::setSaveIntermediateSteps(bool saveSteps)
{
    m_bSaveIntermediateSteps = saveSteps;
}

template <class PointType>
std::vector<MetaRoomUpdateIteration<PointType>> MetaRoom<PointType>::getUpdateIterations()
{
    return m_MetaRoomUpdateIterations;
}

template <class PointType>
 std::pair<pcl::ModelCoefficients::Ptr,bool> MetaRoom<PointType>::getCeilingPrimitive()
{
    std::pair<pcl::ModelCoefficients::Ptr,bool> toRet;
    toRet.first = m_MetaRoomCeilingPrimitive;
    toRet.second = m_MetaRoomCeilingPrimitiveDirection;
    return toRet;
}

template <class PointType>
void MetaRoom<PointType>::setCeilingPrimitive(pcl::ModelCoefficients::Ptr primitive,bool direction)
 {
     m_MetaRoomCeilingPrimitive = primitive;
     m_MetaRoomCeilingPrimitiveDirection = direction;
 }

 template <class PointType>
std::pair<pcl::ModelCoefficients::Ptr,bool> MetaRoom<PointType>::getFloorPrimitive()
{
    std::pair<pcl::ModelCoefficients::Ptr,bool> toRet;
    toRet.first = m_MetaRoomFloorPrimitive;
    toRet.second = m_MetaRoomFloorPrimitiveDirection;
    return toRet;
}

template <class PointType>
void MetaRoom<PointType>::setFloorPrimitive(pcl::ModelCoefficients::Ptr primitive,bool direction)
{
    m_MetaRoomFloorPrimitive = primitive;
    m_MetaRoomFloorPrimitiveDirection = direction;
}

template <class PointType>
std::pair<std::vector<pcl::ModelCoefficients::Ptr>,std::vector<bool> > MetaRoom<PointType>::getWallPrimitives()
{
    std::pair<std::vector<pcl::ModelCoefficients::Ptr>,std::vector<bool> > toRet;
    for (size_t i=0; i<m_vMetaRoomWallPrimitives.size();i++)
    {
        toRet.first.push_back(m_vMetaRoomWallPrimitives[i]);
        toRet.second.push_back(m_vMetaRoomWallPrimitivesDirections[i]);
    }
    return toRet;
}

template <class PointType>
void MetaRoom<PointType>::setWallPrimitives(std::vector<pcl::ModelCoefficients::Ptr> primitives ,std::vector<bool> directions)
{
    for (size_t i=0; i<primitives.size();i++)
    {
        m_vMetaRoomWallPrimitives.push_back(primitives[i]);
        m_vMetaRoomWallPrimitivesDirections.push_back(directions[i]);
    }
}

template <class PointType>
std::pair<std::vector<pcl::ModelCoefficients::Ptr>,std::vector<bool> > MetaRoom<PointType>::getBoundingPrimitives()
{

    std::pair<std::vector<pcl::ModelCoefficients::Ptr>,std::vector<bool> > toRet;

    toRet.first.push_back(this->m_MetaRoomCeilingPrimitive);
    toRet.second.push_back(this->m_MetaRoomCeilingPrimitiveDirection);

    toRet.first.push_back(this->m_MetaRoomFloorPrimitive);
    toRet.second.push_back(this->m_MetaRoomFloorPrimitiveDirection);

    for (size_t i=0; i<m_vMetaRoomWallPrimitives.size();i++)
    {
        toRet.first.push_back(m_vMetaRoomWallPrimitives[i]);
        toRet.second.push_back(m_vMetaRoomWallPrimitivesDirections[i]);
    }

    return toRet;
}

template <class PointType>
void MetaRoom<PointType>::addUpdateIteration(MetaRoomUpdateIteration<PointType> newIteration)
{
    m_MetaRoomUpdateIterations.push_back(newIteration);
}

template <class PointType>
void MetaRoom<PointType>::setConsistencyUpdateCloud(CloudPtr cons)
{
    *m_ConsistencyUpdateCloud = *cons;
    m_ConsistencyUpdateCloudLoaded = true;
}

template <class PointType>
void MetaRoom<PointType>::setConsistencyUpdateCloud(std::string cons)
{
    m_ConsistencyUpdateCloudFilename = cons;
    m_ConsistencyUpdateCloudLoaded = false;
}

template <class PointType>
bool MetaRoom<PointType>::getConsistencyUpdateCloudLoaded()
{
    return m_ConsistencyUpdateCloudLoaded;
}

template <class PointType>
std::string MetaRoom<PointType>::getConsistencyUpdateCloudFilename()
{
    return m_ConsistencyUpdateCloudFilename;
}

template <class PointType>
typename pcl::PointCloud<PointType>::Ptr MetaRoom<PointType>::getConsistencyUpdateCloud()
{
    if (!m_ConsistencyUpdateCloudLoaded)
    {
        // first load the consistency update cloud
        std::cout<<"Loading the consistency update cloud "<<m_ConsistencyUpdateCloudFilename<<std::endl;
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read (m_ConsistencyUpdateCloudFilename, *cloud);
        *m_ConsistencyUpdateCloud = *cloud;
        m_ConsistencyUpdateCloudLoaded = true;
    }
    return m_ConsistencyUpdateCloud;
}

template <class PointType>
tf::Vector3 MetaRoom<PointType>::getSensorOrigin()
{
    return this->m_SensorOrigin;
}

template <class PointType>
void MetaRoom<PointType>::setSensorOrigin(tf::Vector3 so)
{
    this->m_SensorOrigin = so;
}

template <class PointType>
bool    MetaRoom<PointType>::updateMetaRoom(SemanticRoom<PointType>& aRoom, std::string savePath)
{
    // check if meta room is initialized
    if (!this->m_CompleteRoomCloudLoaded && this->m_CompleteRoomCloudFilename == "")
    {
        // meta room not initialized -> initialize it with this room
        this->setCompleteRoomCloud(aRoom.getCompleteRoomCloud());
        this->setCentroid(aRoom.getCentroid());

        ROS_INFO_STREAM("Initializing metaroom with room");
        ROS_INFO_STREAM("Room log name "<<aRoom.getRoomLogName());
        ROS_INFO_STREAM("Room run number "<<aRoom.getRoomRunNumber());
//            ROS_INFO_STREAM("Room centroid "<<aRoom.getCentroid());
        std::vector<tf::StampedTransform> roomTransforms = aRoom.getIntermediateCloudTransforms();
        if (roomTransforms.size() == 0)
        {
            ROS_INFO_STREAM("No intermediate transforms saved. Sensor origin will be set as the origin");
            this->m_SensorOrigin = tf::Vector3(0.0,0.0,0.0);
        } else {

            tf::StampedTransform middleTransform = roomTransforms[(int)floor(roomTransforms.size()/2)];
            this->m_SensorOrigin = middleTransform.getOrigin();
        }
        // filter down metaroom point cloud
        CloudPtr cloud_filtered = MetaRoom<PointType>::downsampleCloud(this->getCompleteRoomCloud()->makeShared());

        this->setDeNoisedRoomCloud(cloud_filtered);
        this->setInteriorRoomCloud(cloud_filtered);

        aRoom.setDeNoisedRoomCloud(cloud_filtered);
        aRoom.setInteriorRoomCloud(cloud_filtered);


        QString rootFolderPath;

        if (savePath == "")
        {
            // save room in the correct folder
            QString roomXml(aRoom.getCompleteRoomCloudFilename().c_str());
            int date = roomXml.indexOf("201");
            rootFolderPath = roomXml.left(date);
        } else {
            rootFolderPath = QString(savePath.c_str()) + QString("/");
        }

        ROS_INFO_STREAM("Initializeing room xml parser with root folder "<<rootFolderPath.toStdString());
        SemanticRoomXMLParser<PointType> parser(rootFolderPath.toStdString());
        parser.saveRoomAsXML(aRoom);


        return true;
    }

    // update with semantic room

    // check that the centroids are close enough and that the room is actually an instance of this metaroom
    double centroidDistance = pcl::distances::l2(this->getCentroid(),aRoom.getCentroid());
    ROS_INFO_STREAM("Comparing metaroom centroid with room centroid. Distance: "<<centroidDistance);
    if (! (centroidDistance < ROOM_CENTROID_DISTANCE) )
    {
        // this room is not a match for this metaroom
        ROS_INFO_STREAM("Cannot update metaroom with this room instance. Metaroom centroid: "<<this->getCentroid()<<" Room centroid: "<<aRoom.getCentroid());
        return false;
    }


    // check if semantic room needs to be transformed into the metaroom frame of reference
    Eigen::Matrix4f roomTransform = aRoom.getRoomTransform();
    if (roomTransform == Eigen::Matrix4f::Identity())
    { // identity transform -> needs to be updated
        ROS_INFO_STREAM("Transforming semantic room into metaroom frame of reference.");
        CloudPtr output(new Cloud);
        Eigen::Matrix4f finalTransform;
        CloudPtr roomCloud = aRoom.getCompleteRoomCloud();

        CloudPtr transformedRoomCloud(new Cloud);
        transformedRoomCloud = NdtRegistration<PointType>::registerClouds(roomCloud, this->getCompleteRoomCloud(),finalTransform);

        ROS_INFO_STREAM("Room alignment complete.");

        // extract noise and re-align rooms (hoping to get a better alignment without the noise outside the walls).
        CloudPtr roomDownsampledCloud = MetaRoom<PointType>::downsampleCloud(transformedRoomCloud);

        aRoom.setDeNoisedRoomCloud(roomDownsampledCloud);

        aRoom.setRoomTransform(finalTransform);

        // Update room XML file to reflect new transformation, and new point clouds
        ROS_INFO_STREAM("Updating room xml with new transform to metaroom.");
        aRoom.setInteriorRoomCloud(roomDownsampledCloud);

        QString rootFolderPath;

        if (savePath == "")
        {
            // save room in the correct folder
            QString roomXml(aRoom.getCompleteRoomCloudFilename().c_str());
            int date = roomXml.indexOf("201");
            rootFolderPath = roomXml.left(date);
        } else {
            rootFolderPath = QString(savePath.c_str()) + QString("/");
        }

        ROS_INFO_STREAM("Initializeing room xml parser with root folder "<<rootFolderPath.toStdString());
        SemanticRoomXMLParser<PointType> parser(rootFolderPath.toStdString());
        parser.saveRoomAsXML(aRoom);
    }

    if (!m_bUpdateMetaroom)
    {
        // stop here, don't update the metaroom with the room observation
        return true;
    }

    // compute differences between the two (aligned) points clouds
    {
        CloudPtr differenceMetaRoomToRoom(new Cloud);
        CloudPtr differenceRoomToMetaRoom(new Cloud);
        CloudPtr differenceMetaRoomToRoomFiltered(new Cloud);
        CloudPtr differenceRoomToMetaRoomFiltered(new Cloud);

        // the room cloud is transformed by default
        CloudPtr transformedRoomCloud(new Cloud);
//            pcl::transformPointCloud (*aRoom.getInteriorRoomCloud(), *transformedRoomCloud, aRoom.getRoomTransform());
        transformedRoomCloud = aRoom.getInteriorRoomCloud();

        // compute the differences
        pcl::SegmentDifferences<PointType> segment;
        segment.setInputCloud(this->getInteriorRoomCloud());
        segment.setTargetCloud(transformedRoomCloud);
        segment.setDistanceThreshold(0.001);

        typename Tree::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (transformedRoomCloud);
        segment.setSearchMethod(tree);

        segment.segment(*differenceMetaRoomToRoom);

        segment.setInputCloud(transformedRoomCloud);
        segment.setTargetCloud(this->getInteriorRoomCloud());
        tree->setInputCloud(this->getInteriorRoomCloud());

        segment.segment(*differenceRoomToMetaRoom);

        // apply a statistical noise removal filter
//        pcl::StatisticalOutlierRemoval<PointType> sor;
//        sor.setInputCloud (differenceRoomToMetaRoom);
//        sor.setMeanK (50);
//        sor.setStddevMulThresh (1.0);
//        sor.filter (*differenceRoomToMetaRoomFiltered);

//        sor.setInputCloud (differenceMetaRoomToRoom);
//        sor.filter (*differenceMetaRoomToRoomFiltered);
        *differenceRoomToMetaRoomFiltered = *differenceRoomToMetaRoom;
        *differenceMetaRoomToRoomFiltered = *differenceMetaRoomToRoom;

        // Cluster objects
//        std::vector<CloudPtr> vDifferenceMetaRoomToRoomClusters = this->clusterPointCloud(differenceMetaRoomToRoomFiltered,0.05,65,100000);
//        std::vector<CloudPtr> vDifferenceRoomToMetaRoomClusters = this->clusterPointCloud(differenceRoomToMetaRoomFiltered,0.05,65,100000);


//        // filter clusters based on distance
//        double maxDistance = 3.0; // max of 3 meters
//        this->filterClustersBasedOnDistance(vDifferenceRoomToMetaRoomClusters, maxDistance);
//        this->filterClustersBasedOnDistance(vDifferenceMetaRoomToRoomClusters, maxDistance);

//        // check cluster occlusions
//        OcclusionChecker<PointType> occlusionChecker;
//        occlusionChecker.setSensorOrigin(m_SensorOrigin);
//        std::vector<CloudPtr> clustersToBeAdded = occlusionChecker.checkOcclusions(vDifferenceMetaRoomToRoomClusters, vDifferenceRoomToMetaRoomClusters);
//        ROS_INFO_STREAM("Finished checking occlusions.");

//        // add all the clusters together for subtraction
//        int pointsAdded =0, pointsRemoved = 0;
//        CloudPtr allClusters(new Cloud());
//        for (size_t i=0; i<vDifferenceMetaRoomToRoomClusters.size();i++)
//        {
//            *allClusters += *vDifferenceMetaRoomToRoomClusters[i];
//            pointsRemoved += vDifferenceMetaRoomToRoomClusters[i]->points.size();
//        }

//        // update metaroom by subracting the difference to the room
//        CloudPtr updatedMetaRoomCloud(new Cloud());
//        segment.setInputCloud(this->getInteriorRoomCloud());
//        segment.setTargetCloud(allClusters);
//        segment.segment(*updatedMetaRoomCloud);

//        CloudPtr cloudToBeAdded(new Cloud());
//        // add ocluded clusters from the room
//        for (size_t i=0; i<clustersToBeAdded.size(); i++)
//        {
//            *cloudToBeAdded += *clustersToBeAdded[i];
//            pointsAdded += clustersToBeAdded[i]->points.size();
//        }
//        *updatedMetaRoomCloud += *cloudToBeAdded;

        //        ROS_INFO_STREAM("Metaroom update. Points removed: "<<pointsRemoved<<"   Points added: "<<pointsAdded);

        OcclusionChecker<PointType> occlusionChecker;
        occlusionChecker.setSensorOrigin(m_SensorOrigin);
        typename OcclusionChecker<PointType>::occluded_points occlusions;
        occlusions = occlusionChecker.checkOcclusions(differenceMetaRoomToRoom,differenceRoomToMetaRoom, 720 );

        CloudPtr updatedMetaRoomCloud(new Cloud());
        segment.setInputCloud(this->getInteriorRoomCloud());
        segment.setTargetCloud(occlusions.toBeRemoved);
        segment.segment(*updatedMetaRoomCloud);
        *updatedMetaRoomCloud += *occlusions.toBeAdded;
        ROS_INFO_STREAM("Metaroom update. Points removed: "<<occlusions.toBeRemoved->points.size()<<"   Points added: "<<occlusions.toBeAdded->points.size());

        this->setInteriorRoomCloud(updatedMetaRoomCloud);

        if (m_bSaveIntermediateSteps)
        {
            if ((differenceMetaRoomToRoomFiltered->points.size()!=0) && (differenceRoomToMetaRoomFiltered->points.size() != 0))
            {
                MetaRoomUpdateIteration<PointType> updateIteration;
                updateIteration.roomLogName = aRoom.getRoomLogName();
                updateIteration.roomRunNumber = aRoom.getRoomRunNumber();
                updateIteration.setDifferenceMetaRoomToRoom(differenceMetaRoomToRoomFiltered);
                updateIteration.setDifferenceRoomToMetaRoom(differenceRoomToMetaRoomFiltered);
//                updateIteration.setClustersToBeAdded(cloudToBeAdded);
//                updateIteration.setClustersToBeRemoved(allClusters);
                updateIteration.setClustersToBeAdded(occlusions.toBeAdded);
                updateIteration.setClustersToBeRemoved(occlusions.toBeRemoved);
                updateIteration.setMetaRoomInteriorCloud(updatedMetaRoomCloud);
                m_MetaRoomUpdateIterations.push_back(updateIteration);
            }
        }

    }

}

template <class PointType>
std::vector<typename pcl::PointCloud<PointType>::Ptr> MetaRoom<PointType>::clusterPointCloud(CloudPtr input_cloud, double tolerance, int min_cluster_size, int max_cluster_size)
{
    typename Tree::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (input_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_cloud);
    ec.extract (cluster_indices);

    std::vector<CloudPtr> toRet;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        CloudPtr cloud_cluster (new Cloud());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (input_cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;


        toRet.push_back(cloud_cluster);

        j++;
    }

    return toRet;
}

template <class PointType>
void MetaRoom<PointType>::filterClustersBasedOnDistance(std::vector<CloudPtr>& clusters, double maxDistance)
{
    typename std::vector<CloudPtr>::iterator cluster_iterator = clusters.begin();

    while (!(cluster_iterator == clusters.end()))
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*(*cluster_iterator), centroid);

        Eigen::Vector4f roomCenter(m_SensorOrigin.x(), m_SensorOrigin.y(), m_SensorOrigin.z(),0.0);

        double distance = pcl::distances::l2(centroid,roomCenter);
        if (distance > maxDistance)
        {
            cluster_iterator = clusters.erase(cluster_iterator);
        } else {
            cluster_iterator++;
        }
    }
}

template <class PointType>
typename MetaRoom<PointType>::CloudPtr MetaRoom<PointType>::downsampleCloud(CloudPtr input, double leafSize)
{
    ROS_INFO_STREAM("PointCloud before filtering has: " << input->points.size () << " data points.");

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<PointType> vg;
    CloudPtr cloud_filtered (new Cloud);
    vg.setInputCloud (input);
    vg.setLeafSize (leafSize, leafSize, leafSize);
    vg.filter (*cloud_filtered);
    ROS_INFO_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points.");

    return cloud_filtered;
}
