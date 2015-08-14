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

        // sort observations according to patrol run and then room number
        sort(toRet.begin(), toRet.end(),
             [](const std::string& a, const std::string& b )
        {
            std::string patrol_string = "patrol_run_";
            std::string room_string = "room_";
            size_t p_pos_1 = a.find(patrol_string);
            size_t r_pos_1 = a.find(room_string) - 1; // remove the / before the room_
            size_t sl_pos_1 = a.find("/",r_pos_1+1);

            size_t p_pos_2 = b.find(patrol_string);
            size_t r_pos_2 = b.find(room_string) - 1; // remove the / before the room_
            size_t sl_pos_2 = b.find("/",r_pos_2+1);

            // just in case we have some different folder structure (shouldn't happen)
            if ((p_pos_1 == std::string::npos) || (r_pos_1 == std::string::npos) || (sl_pos_1 == std::string::npos) ||
                    (p_pos_2 == std::string::npos) || (r_pos_2 == std::string::npos) || (sl_pos_2 == std::string::npos))
            {
                return a<b;
            }

            std::string p_1 = a.substr(p_pos_1 + patrol_string.length(), r_pos_1 - (p_pos_1 + patrol_string.length()));
            std::string r_1 = a.substr(r_pos_1 + 1 + room_string.length(), sl_pos_1 - (r_pos_1 + 1 + room_string.length()));
//            std::cout<<"Patrol 1: "<<p_1<<"  room 1: "<<r_1<<std::endl;
            std::string p_2 = b.substr(p_pos_2 + patrol_string.length(), r_pos_2 - (p_pos_2 + patrol_string.length()));
            std::string r_2 = b.substr(r_pos_2 + 1 + room_string.length(), sl_pos_2 - (r_pos_2 + 1 + room_string.length()));
//            std::cout<<"Patrol 2: "<<p_2<<"  room 2: "<<r_2<<std::endl;

            if (stoi(p_1) == stoi(p_2)){
                if (stoi(r_1) == stoi(r_2)){
                    return a<b;
                } else {
                    return stoi(r_1) < stoi(r_2);
                }
            } else {
                return stoi(p_1) < stoi(p_2);
            }
        }
             );


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

        // sort observations according to patrol run and then room number
        std::vector<std::string> matchingObservations = waypointToSweepsMap[waypoint];

        sort(matchingObservations.begin(), matchingObservations.end(),
             [](const std::string& a, const std::string& b )
        {            
            std::string patrol_string = "patrol_run_";
            std::string room_string = "room_";
            std::string date_string = "YYYYMMDD";
            size_t p_pos_1 = a.find(patrol_string);
            size_t r_pos_1 = a.find(room_string) - 1; // remove the / before the room_
            size_t sl_pos_1 = a.find("/",r_pos_1+1);

            size_t p_pos_2 = b.find(patrol_string);
            size_t r_pos_2 = b.find(room_string) - 1; // remove the / before the room_
            size_t sl_pos_2 = b.find("/",r_pos_2+1);

            // just in case we have some different folder structure (shouldn't happen)
            if ((p_pos_1 == std::string::npos) || (r_pos_1 == std::string::npos) || (sl_pos_1 == std::string::npos) ||
                    (p_pos_2 == std::string::npos) || (r_pos_2 == std::string::npos) || (sl_pos_2 == std::string::npos))
            {
                return a<b;
            }

            std::string d_1 = a.substr(p_pos_1 - date_string.length() -1, date_string.length());
            std::string p_1 = a.substr(p_pos_1 + patrol_string.length(), r_pos_1 - (p_pos_1 + patrol_string.length()));
            std::string r_1 = a.substr(r_pos_1 + 1 + room_string.length(), sl_pos_1 - (r_pos_1 + 1 + room_string.length()));
//            std::cout<<"Patrol 1: "<<p_1<<"  room 1: "<<r_1<<std::endl;
//            std::cout<<"Patrol 1: "<<p_1<<"  room 1: "<<r_1<<"  date 1 "<<d_1<<std::endl;
            std::string d_2 = b.substr(p_pos_2 - date_string.length() -1, date_string.length());
            std::string p_2 = b.substr(p_pos_2 + patrol_string.length(), r_pos_2 - (p_pos_2 + patrol_string.length()));
            std::string r_2 = b.substr(r_pos_2 + 1 + room_string.length(), sl_pos_2 - (r_pos_2 + 1 + room_string.length()));
//            std::cout<<"Patrol 2: "<<p_2<<"  room 2: "<<r_2<<"  date 2 "<<d_2<<std::endl;
//            std::cout<<"Patrol 2: "<<p_2<<"  room 2: "<<r_2<<std::endl;

            if (d_1 == d_2){
            if (stoi(p_1) == stoi(p_2)){
                if (stoi(r_1) == stoi(r_2)){
                    return a<b;
                } else {
                    return stoi(r_1) < stoi(r_2);
                }
            } else {
                return stoi(p_1) < stoi(p_2);
            }
        } else {
             return d_1<d_2;
            }
        }
             );


        return matchingObservations;

    }

    /********************************************** DYNAMIC CLUSTER UTILITIES ****************************************************************************************/
    ///* The default parameters are the same as during the metaroom update
    template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadDynamicClustersFromSingleSweep(std::string sweepXmlPath, bool verbose=false, double tolerance = 0.05, int min_cluster_size = 75, int max_cluster_size=50000)
    {
        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> toRet;

        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"RoomDynamicClusters"},verbose);

        // split the dynamic cluster point cloud into individual clusters
        // it would be easier to save the split clusters directly, as opposed to combined in one point clouds, but more work
        // so the combined clusters will be split again into individual clusters here

        if (sweep.dynamicClusterCloud->points.size() > 0)
        {
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
        } else {
            ROS_INFO_STREAM("Sweep "<<sweepXmlPath<<" doesn't have the RoomDynamicClusters node set in the xml file");
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
            toRet.push_back(loadDynamicClustersFromSingleSweep<PointType>(sweepsAtWaypoint[i], verbose, tolerance, min_cluster_size, max_cluster_size));
         }

         return toRet;
    }

    /********************************************** LABELLED DATA UTILITIES ****************************************************************************************/template <class PointType>
    LabelledData<PointType> loadLabelledDataFromSingleSweep(std::string sweepXmlPath, bool verbose = false)
    {
        LabelledData<PointType> toRet;

        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"RoomCompleteCloud", "RoomIntermediateCloud"},false, false);

        // sweep data
        toRet.completeCloud = sweep.completeRoomCloud;
        toRet.sweepCenter = sweep.vIntermediateRoomCloudTransformsRegistered[sweep.vIntermediateRoomCloudTransformsRegistered.size()/2].getOrigin();
        toRet.transformToGlobal = sweep.vIntermediateRoomCloudTransforms[0];
        toRet.waypoint = sweep.roomWaypointId;
        toRet.sweepTime = sweep.roomLogStartTime;

        // get labelled objects
        unsigned found = sweepXmlPath.find_last_of("/");
        std::string base_path = sweepXmlPath.substr(0,found+1);
        QStringList xmlFiles = QDir(base_path.c_str()).entryList(QStringList("*label*.pcd"));

        for (size_t k=0; k<xmlFiles.size(); k++)
        {
            std::string label_file = base_path+xmlFiles[k].toStdString();
            label_file[label_file.size()-1] = 't';
            label_file[label_file.size()-2] = 'x';
            label_file[label_file.size()-3] = 't';
            std::ifstream label_stream; label_stream.open(label_file);
            std::string label;
            label_stream >> label;
            label_stream.close();

            pcl::PCDReader reader;
            boost::shared_ptr<pcl::PointCloud<PointType>> cloud (new pcl::PointCloud<PointType>);
            reader.read (base_path+xmlFiles[k].toStdString(), *cloud);

            if (!cloud->points.size()){
                continue;
            }

            toRet.objectClouds.push_back(cloud);
            toRet.objectLabels.push_back(label);
        }

        return toRet;
    }

