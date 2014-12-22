    /********************************************** MERGED CLOUD UTILITIES ****************************************************************************************/
    template <class PointType>
    boost::shared_ptr<pcl::PointCloud<PointType>> loadMergedCloudFromSingleSweep(std::string sweepXmlPath, bool verbose=false)
    {
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"RoomCompleteCloud"},verbose);

        return sweep.completeRoomCloud;
    }

    template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadMergedCloudFromMultipleSweeps(std::string folderPath, bool verbose=false)
    {
        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> toRet;

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>{"RoomCompleteCloud"}, verbose);
            toRet.push_back(sweep.completeRoomCloud);
        }

        return toRet;
    }

    template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadMergedCloudForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose=false)
    {
        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> toRet;

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        // first construct waypoint id to sweep xml map
        std::map<std::string, std::vector<std::string>> waypointToSweepsMap;
        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), verbose);
            waypointToSweepsMap[sweep.roomWaypointId].push_back(sweep_xmls[i].roomXmlFile);
        }

        auto sweepsAtWaypoint = waypointToSweepsMap[waypoint];

        for (size_t i=0; i<sweepsAtWaypoint.size(); i++)
        {
            auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepsAtWaypoint[i], std::vector<std::string>{"RoomCompleteCloud"}, verbose);
            toRet.push_back(sweep.completeRoomCloud);
        }

        return toRet;
    }

    /********************************************** INTERMEDIATE CLOUD UTILITIES ****************************************************************************************/

    template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadIntermediateCloudsFromSingleSweep(std::string sweepXmlPath, bool verbose=false)
    {
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"RoomIntermediateCloud"},verbose);

        return sweep.vIntermediateRoomClouds;
    }

    template <class PointType>
    IntermediateCloudCompleteData<PointType> loadIntermediateCloudsCompleteDataFromSingleSweep(std::string sweepXmlPath, bool verbose=false)
    {
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"RoomIntermediateCloud"},verbose);

        IntermediateCloudCompleteData<PointType> toRet;
        toRet.vIntermediateRoomClouds = sweep.vIntermediateRoomClouds;
        toRet.vIntermediateRoomCloudTransforms = sweep.vIntermediateRoomCloudTransforms;
        toRet.vIntermediateRoomCloudCamParams = sweep.vIntermediateRoomCloudCamParams;
        toRet.vIntermediateRGBImages = sweep.vIntermediateRGBImages;
        toRet.vIntermediateDepthImages = sweep.vIntermediateDepthImages;

        return toRet;
    }

    template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadIntermediateCloudsFromMultipleSweeps(std::string folderPath, bool verbose=false)
    {
        std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> toRet;

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            auto intClouds = loadIntermediateCloudsFromSingleSweep<PointType>(sweep_xmls[i],verbose);
            toRet.push_back(intClouds);
        }

        return toRet;
    }

    template <class PointType>
    std::vector<IntermediateCloudCompleteData<PointType>>  loadIntermediateCloudsCompleteDataFromMultipleSweeps(std::string folderPath, bool verbose=false)
    {
        std::vector<IntermediateCloudCompleteData<PointType>> toRet;

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            auto intCloudsComplete = loadIntermediateCloudsCompleteDataFromSingleSweep<PointType>(sweep_xmls[i],verbose);
            toRet.push_back(intCloudsComplete);
        }

        return toRet;
    }

    template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadIntermediateCloudsForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose=false)
    {
        std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> toRet;

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        // first construct waypoint id to sweep xml map
        std::map<std::string, std::vector<std::string>> waypointToSweepsMap;
        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), verbose);
            waypointToSweepsMap[sweep.roomWaypointId].push_back(sweep_xmls[i].roomXmlFile);
        }

        auto sweepsAtWaypoint = waypointToSweepsMap[waypoint];

        for (size_t i=0; i<sweepsAtWaypoint.size(); i++)
        {
            auto intClouds = loadIntermediateCloudsFromSingleSweep<PointType>(sweepsAtWaypoint[i],verbose);
            toRet.push_back(intClouds);
        }

        return toRet;
    }

        template <class PointType>
        std::vector<IntermediateCloudCompleteData<PointType>> loadIntermediateCloudsCompleteDataForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose=false)
    {
        std::vector<IntermediateCloudCompleteData<PointType>> toRet;

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        // first construct waypoint id to sweep xml map
        std::map<std::string, std::vector<std::string>> waypointToSweepsMap;
        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), verbose);
            waypointToSweepsMap[sweep.roomWaypointId].push_back(sweep_xmls[i].roomXmlFile);
        }

        auto sweepsAtWaypoint = waypointToSweepsMap[waypoint];

        for (size_t i=0; i<sweepsAtWaypoint.size(); i++)
        {
            auto intCloudsComplete = loadIntermediateCloudsCompleteDataFromSingleSweep<PointType>(sweepsAtWaypoint[i],verbose);
            toRet.push_back(intCloudsComplete);
        }

        return toRet;
    }

    /********************************************** INTERMEDIATE POSITION IMAGES UTILITIES ****************************************************************************************/
    template <class PointType>
    std::vector<typename SimpleXMLParser<PointType>::IntermediatePositionImages> loadIntermediatePositionImagesFromSingleSweep(std::string sweepXmlPath, bool verbose=false)
    {
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"IntermediatePosition"},verbose);

        return sweep.vIntermediatePositionImages;
    }

    template <class PointType>
    std::vector<std::vector<typename SimpleXMLParser<PointType>::IntermediatePositionImages>> loadIntermediatePositionImagesFromMultipleSweeps(std::string folderPath, bool verbose=false)
    {
        std::vector<std::vector<typename SimpleXMLParser<PointType>::IntermediatePositionImages>> toRet;

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i], std::vector<std::string>{"IntermediatePosition"},verbose);
            toRet.push_back(sweep.vIntermediatePositionImages);
        }

        return toRet;
    }

    template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadIntermediatePositionImagesForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose=false)
    {
        std::vector<std::vector<typename SimpleXMLParser<PointType>::IntermediatePositionImages>> toRet;

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        // first construct waypoint id to sweep xml map
        std::map<std::string, std::vector<std::string>> waypointToSweepsMap;
        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), verbose);
            waypointToSweepsMap[sweep.roomWaypointId].push_back(sweep_xmls[i].roomXmlFile);
        }

        auto sweepsAtWaypoint = waypointToSweepsMap[waypoint];

        for (size_t i=0; i<sweepsAtWaypoint.size(); i++)
        {
            auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepsAtWaypoint[i], std::vector<std::string>{"IntermediatePosition"},verbose);
            toRet.push_back(sweep.vIntermediatePositionImages);
        }

        return toRet;
    }


    /********************************************** SWEEP XML UTILITIES ****************************************************************************************/
    template <class PointType>
    std::vector<std::string>  getSweepXmls(std::string folderPath, bool verbose = false)
    {
        std::vector<std::string> toRet;

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            toRet.push_back(sweep_xmls[i].roomXmlFile);
        }

        return toRet;
    }

    template <class PointType>
    std::vector<std::string>  getSweepXmlsForTopologicalWaypoint(std::string folderPath, std::string waypoint, bool verbose= false)
    {

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        // first construct waypoint id to sweep xml map
        std::map<std::string, std::vector<std::string>> waypointToSweepsMap;
        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), verbose);
            waypointToSweepsMap[sweep.roomWaypointId].push_back(sweep_xmls[i].roomXmlFile);
        }

        return waypointToSweepsMap[waypoint];

    }

    /********************************************** DYNAMIC CLUSTER UTILITIES ****************************************************************************************/
    ///* The default parameters are the same as during the metaroom update
    template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadDynamicClustersFromSingleSweep(std::string sweepXmlPath, bool verbose=false, double tolerance = 0.05, int min_cluster_size = 75, int max_cluster_size=50000)
    {

        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"RoomDynamicClusters"},verbose);

        // split the dynamic cluster point cloud into individual clusters
        // it would be easier to save the split clusters directly, as opposed to combined in one point clouds, but more work
        // so the combined clusters will be split again into individual clusters here

        typename pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (sweep.dynamicClusterCloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance (tolerance);
        ec.setMinClusterSize (min_cluster_size);
        ec.setMaxClusterSize (max_cluster_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (sweep.dynamicClusterCloud);
        ec.extract (cluster_indices);

        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> toRet;

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            boost::shared_ptr<pcl::PointCloud<PointType>> cloud_cluster (new pcl::PointCloud<PointType>());
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster->points.push_back (sweep.dynamicClusterCloud->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;


            toRet.push_back(cloud_cluster);

        }

        return toRet;
    }

    template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadDynamicClustersFromMultipleSweeps(std::string folderPath, bool verbose=false, double tolerance = 0.05, int min_cluster_size = 75, int max_cluster_size=50000)
    {
        std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> toRet;

        SimpleSummaryParser summary_parser;
        summary_parser.createSummaryXML(folderPath);
        auto sweep_xmls = summary_parser.getRooms();

        for (size_t i=0; i<sweep_xmls.size(); i++)
        {
            toRet.push_back(loadDynamicClustersFromSingleSweep<PointType>(sweep_xmls[i].roomXmlFile, verbose, tolerance, min_cluster_size, max_cluster_size));
        }

        return toRet;
    }

    template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadDynamicClustersForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose=false, double tolerance = 0.05, int min_cluster_size = 75, int max_cluster_size=50000)
    {
        std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> toRet;

         SimpleSummaryParser summary_parser;
         summary_parser.createSummaryXML(folderPath);
         auto sweep_xmls = summary_parser.getRooms();

         // first construct waypoint id to sweep xml map
         std::map<std::string, std::vector<std::string>> waypointToSweepsMap;
         for (size_t i=0; i<sweep_xmls.size(); i++)
         {
             auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xmls[i].roomXmlFile, std::vector<std::string>(), verbose);
             waypointToSweepsMap[sweep.roomWaypointId].push_back(sweep_xmls[i].roomXmlFile);
         }

         auto sweepsAtWaypoint = waypointToSweepsMap[waypoint];

         for (size_t i=0; i<sweepsAtWaypoint.size(); i++)
         {
            toRet.push_back(loadDynamicClustersFromSingleSweep<PointType>(sweep_xmls[i].roomXmlFile, verbose, tolerance, min_cluster_size, max_cluster_size));
         }

         return toRet;
    }
