/********************************************** MERGED CLOUD UTILITIES ****************************************************************************************/
template <class PointType>
    boost::shared_ptr<pcl::PointCloud<PointType>> loadMergedCloudFromSingleSweep(std::string sweepXmlPath, bool verbose)
{
    auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"RoomCompleteCloud"},verbose);

    return sweep.completeRoomCloud;
}

template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadMergedCloudFromMultipleSweeps(std::string folderPath, bool verbose)
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
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadMergedCloudForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose)
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
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadIntermediateCloudsFromSingleSweep(std::string sweepXmlPath, bool verbose)
{
    auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"RoomIntermediateCloud"},verbose);

    return sweep.vIntermediateRoomClouds;
}

template <class PointType>
    IntermediateCloudCompleteData<PointType> loadIntermediateCloudsCompleteDataFromSingleSweep(std::string sweepXmlPath, bool verbose)
{
    auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"RoomIntermediateCloud"},verbose);

    IntermediateCloudCompleteData<PointType> toRet;
    toRet.vIntermediateRoomClouds = sweep.vIntermediateRoomClouds;
    toRet.vIntermediateRoomCloudTransforms = sweep.vIntermediateRoomCloudTransforms;
    toRet.vIntermediateRoomCloudCamParams = sweep.vIntermediateRoomCloudCamParams;
    toRet.vIntermediateRoomCloudTransformsRegistered = sweep.vIntermediateRoomCloudTransformsRegistered;
    toRet.vIntermediateRoomCloudCamParamsCorrected = sweep.vIntermediateRoomCloudCamParamsCorrected;
    toRet.vIntermediateRGBImages = sweep.vIntermediateRGBImages;
    toRet.vIntermediateDepthImages = sweep.vIntermediateDepthImages;

    return toRet;
}

template <class PointType>
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadIntermediateCloudsFromMultipleSweeps(std::string folderPath, bool verbose)
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
    std::vector<IntermediateCloudCompleteData<PointType>>  loadIntermediateCloudsCompleteDataFromMultipleSweeps(std::string folderPath, bool verbose)
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
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadIntermediateCloudsForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose)
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
        std::vector<IntermediateCloudCompleteData<PointType>> loadIntermediateCloudsCompleteDataForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose)
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
    std::vector<typename SimpleXMLParser<PointType>::IntermediatePositionImages> loadIntermediatePositionImagesFromSingleSweep(std::string sweepXmlPath, bool verbose)
{
    auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(sweepXmlPath, std::vector<std::string>{"IntermediatePosition"},verbose);

    return sweep.vIntermediatePositionImages;
}

template <class PointType>
    std::vector<std::vector<typename SimpleXMLParser<PointType>::IntermediatePositionImages>> loadIntermediatePositionImagesFromMultipleSweeps(std::string folderPath, bool verbose)
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
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadIntermediatePositionImagesForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose)
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
    std::vector<std::string>  getSweepXmls(std::string folderPath, bool verbose)
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
    std::vector<std::string>  getSweepXmlsForTopologicalWaypoint(std::string folderPath, std::string waypoint, bool verbose)
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
template <class PointType>
DynamicObjectData<PointType> loadDynamicObjectFromSingleSweep(std::string objectXmlPath, bool verbose, bool load_cloud ){
    DynamicObjectData<PointType> toRet;
    auto object = SimpleDynamicObjectParser<PointType>::loadDynamicObject(objectXmlPath, verbose, load_cloud);
    toRet.objectCloud = object.m_points;
    toRet.objectLabel = object.m_label;
    toRet.time = object.m_time;
    toRet.vAdditionalViews = object.m_vAdditionalViews;
    toRet.vAdditionalViewsTransforms = object.m_vAdditionalViewsTransforms;
    toRet.intermediateCloud = boost::shared_ptr<pcl::PointCloud<PointType>>(new pcl::PointCloud<PointType>());

    return toRet;
}



template <class PointType>
std::vector<DynamicObjectData<PointType>> loadAllDynamicObjectsFromSingleSweep(std::string sweepFolder, bool verbose, bool load_cloud ){
    std::vector<DynamicObjectData<PointType>> toRet;

    // check if the user provided the sweep xml or the folder
    unsigned found = sweepFolder.find_last_of(".");
    std::string extension = sweepFolder.substr(found+1,3);

    if (extension == "xml") {
        // strip the extension and the file and keep the folder
        found = sweepFolder.find_last_of("/");
        sweepFolder = sweepFolder.substr(0, found+1);
    }

    sweepFolder+="/"; // safety, in case the / is missing
    if (verbose){
        std::cout<<"Looking for dynamic elements in "<<sweepFolder<<std::endl;
    }

    QStringList objectFiles = QDir(sweepFolder.c_str()).entryList(QStringList("*object*.xml"));

    if (verbose){
        std::cout<<"Found "<<objectFiles.size()<<" dynamic objects."<<std::endl;
    }

    for (auto objectFile : objectFiles){
        if (verbose){
            std::cout<<"Now parsing "<<objectFile.toStdString()<<std::endl;
        }
        auto object = loadDynamicObjectFromSingleSweep<PointType>(sweepFolder+objectFile.toStdString(),verbose);
        // load mask
        std::string mask_file = objectFile.toStdString().substr(0,objectFile.toStdString().size()-4) + "_mask.txt";
        std::ifstream mask_stream(sweepFolder+mask_file);
        int index;
        while (mask_stream.is_open() && !mask_stream.eof()){
            mask_stream>>index;
            object.objectScanIndices.push_back(index);
        }

        if (!object.objectScanIndices.size() > 0){
            if (verbose){
                std::cout<<"The mask hasn't been saved for object ' "<<sweepFolder+objectFile.toStdString()<<" Skipping."<<std::endl;
                continue;
            }
        }

        // find intermediate cloud for this object
        std::string sweep_xml = sweepFolder+"room.xml";
        auto completeSweepData = SimpleXMLParser<PointType>::loadRoomFromXML(sweep_xml);

        if (completeSweepData.vIntermediateRoomClouds.size() != 0){
            // first transform the object cloud by the inverse of the room transform
            Eigen::Matrix4f roomTransformInverse = completeSweepData.roomTransform.inverse();
            //                    pcl::transformPointCloud (*object.objectCloud, *object.objectCloud, roomTransformInverse);

            // transform all the intermediate clouds in the map frame
            tf::StampedTransform transformToOrigin = completeSweepData.vIntermediateRoomCloudTransforms[0];
            std::vector<tf::StampedTransform> allTransforms = completeSweepData.vIntermediateRoomCloudTransformsRegistered;

            int min_size = 1000000;
            int best_cloud = -1;

            for (size_t i=0; i< completeSweepData.vIntermediateRoomClouds.size(); i++){
                // check if the mask has been set & filter intermediate cloud (should speed up computation)
                boost::shared_ptr<pcl::PointCloud<PointType>> filteredCloud(new pcl::PointCloud<PointType>());
                if (object.objectScanIndices.size() > 0){
                    for (auto index : object.objectScanIndices){
                        filteredCloud->points.push_back(completeSweepData.vIntermediateRoomClouds[i]->points[index]);
                    }
                } else {
                    if (verbose){
                        std::cout<<"The mask hasn't been saved for object "<<sweepFolder+objectFile.toStdString()<<std::endl;
                    }
                    *filteredCloud = *completeSweepData.vIntermediateRoomClouds[i];
                }

                if (verbose){
                    std::cout<<"Now comparing with intermediate cloud "<<i<<std::endl;
                }

                pcl_ros::transformPointCloud(*filteredCloud, *filteredCloud,allTransforms[i]);
                pcl_ros::transformPointCloud(*filteredCloud, *filteredCloud,transformToOrigin);

                // compute difference
                boost::shared_ptr<pcl::PointCloud<PointType>> difference(new pcl::PointCloud<PointType>());
                pcl::SegmentDifferences<PointType> segment;
                segment.setInputCloud(object.objectCloud);
                segment.setTargetCloud(filteredCloud);
                segment.setDistanceThreshold(0.001);
                segment.segment(*difference);

                if (difference->points.size() != object.objectCloud->points.size()){
                    if (difference->points.size() < min_size) {
                        min_size = difference->points.size();
                        best_cloud = i;
                    }
                }

            }

            if (best_cloud == -1){
                // run again, this time also applying the room transform
                pcl::transformPointCloud (*object.objectCloud, *object.objectCloud, roomTransformInverse);

                // transform all the intermediate clouds in the map frame
                min_size = 1000000;

                for (size_t i=0; i< completeSweepData.vIntermediateRoomClouds.size(); i++){
                    // check if the mask has been set & filter intermediate cloud (should speed up computation)
                    boost::shared_ptr<pcl::PointCloud<PointType>> filteredCloud(new pcl::PointCloud<PointType>());
                    if (object.objectScanIndices.size() > 0){
                        for (auto index : object.objectScanIndices){
                            filteredCloud->points.push_back(completeSweepData.vIntermediateRoomClouds[i]->points[index]);
                        }
                    } else {
                        if (verbose){
                            std::cout<<"The mask hasn't been saved for object "<<sweepFolder+objectFile.toStdString()<<std::endl;
                        }
                        *filteredCloud = *completeSweepData.vIntermediateRoomClouds[i];
                    }

                    if (verbose){
                        std::cout<<"Now comparing with intermediate cloud "<<i<<std::endl;
                    }

                    pcl_ros::transformPointCloud(*filteredCloud, *filteredCloud,allTransforms[i]);
                    pcl_ros::transformPointCloud(*filteredCloud, *filteredCloud,transformToOrigin);

                    // compute difference
                    boost::shared_ptr<pcl::PointCloud<PointType>> difference(new pcl::PointCloud<PointType>());
                    pcl::SegmentDifferences<PointType> segment;
                    segment.setInputCloud(object.objectCloud);
                    segment.setTargetCloud(filteredCloud);
                    segment.setDistanceThreshold(0.001);
                    segment.segment(*difference);

                    if (difference->points.size() != object.objectCloud->points.size()){
                        if (difference->points.size() < min_size) {
                            min_size = difference->points.size();
                            best_cloud = i;
                        }
                    }
                }
            }

            if (best_cloud!=-1){

                object.intermediateCloud = completeSweepData.vIntermediateRoomClouds[best_cloud];
                object.transformToGlobal = transformToOrigin;
                object.calibratedTransform = allTransforms[best_cloud];
                // compute cv images from intermediate cloud
                auto images = SimpleXMLParser<PointType>::createRGBandDepthFromPC(object.intermediateCloud);
                object.objectRGBImage = images.first;
                object.objectDepthImage = images.second;

                if (verbose){
                    std::cout<<"Matching intermediate cloud found. Cloud number "<<best_cloud<<std::endl;
                }
            }

            if (!object.intermediateCloud->points.size()){
                ROS_ERROR_STREAM("Intermediate cloud couldn't be found for object "<<sweepFolder+objectFile.toStdString());
            }
        } else {
            ROS_ERROR_STREAM("Sweep XML "<<sweep_xml<<" doesn't contain intermediate cloud data. Cannot find the intermediate cloud where the dynamic object came from.");
        }


        toRet.push_back(object);
    }

    //        p->removeAllPointClouds();

    return toRet;
}



///* The default parameters are the same as during the metaroom update
template <class PointType>
    std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> loadDynamicClustersFromSingleSweep(std::string sweepXmlPath, bool verbose, double tolerance, int min_cluster_size, int max_cluster_size)
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
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadDynamicClustersFromMultipleSweeps(std::string folderPath, bool verbose, double tolerance, int min_cluster_size, int max_cluster_size)
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
    std::vector<std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>> loadDynamicClustersForTopologicalWaypoint(std::string folderPath, std::string waypoint,bool verbose, double tolerance, int min_cluster_size, int max_cluster_size)
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
    LabelledData<PointType> loadLabelledDataFromSingleSweep(std::string sweepXmlPath, bool verbose)
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
    QStringList imageFiles = QDir(base_path.c_str()).entryList(QStringList("*object*.jpg"));
    QStringList maskFiles = QDir(base_path.c_str()).entryList(QStringList("*label*.jpg"));

    if (xmlFiles.size() != imageFiles.size())
    {
        ROS_INFO_STREAM("In " << sweepXmlPath << " found different number of labels and object images.");
    }

    for (size_t k=0; k<xmlFiles.size(); k++)
    {
        // get the frame number of the label
        std::string label_name = xmlFiles[k].toStdString();
        std::string number_string = label_name.substr(4, 4);
        size_t scan_number = std::stoi(number_string);

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

        cv::Mat image = cv::imread(base_path+imageFiles[k].toStdString());
        cv::Mat mask_color = cv::imread(base_path+maskFiles[k].toStdString());
        cv::Mat mask;
        cv::cvtColor(mask_color, mask, CV_BGR2GRAY);
        mask = mask > 200;

        toRet.objectClouds.push_back(cloud);
        toRet.objectImages.push_back(image);
        toRet.objectMasks.push_back(mask);
        toRet.objectLabels.push_back(label);
        toRet.objectScanIndices.push_back(scan_number);
    }

    return toRet;
}
