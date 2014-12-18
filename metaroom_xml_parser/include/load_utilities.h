#include "simple_summary_parser.h"
#include "simple_xml_parser.h"

namespace semantic_map_load_utilties
{

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
    struct IntermediateCloudCompleteData
    {
        std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>  vIntermediateRoomClouds;
        std::vector<tf::StampedTransform>                           vIntermediateRoomCloudTransforms;
        std::vector<image_geometry::PinholeCameraModel>             vIntermediateRoomCloudCamParams;
        std::vector<cv::Mat>                                        vIntermediateRGBImages; // type CV_8UC3
        std::vector<cv::Mat>                                        vIntermediateDepthImages; // type CV_16UC1
    };

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

}
