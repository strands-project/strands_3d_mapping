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


}
