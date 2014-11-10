#include "simpleXMLparser.h"
#include "simpleSummaryParser.h"

typedef pcl::PointXYZRGB PointType;

typedef typename SimpleSummaryParser<PointType>::EntityStruct Entities;

using namespace std;

int main(int argc, char** argv)
{
    SimpleSummaryParser<PointType> summary_parser("/home/cvapdemo/Data/index.xml");
    summary_parser.createSummaryXML("/home/cvapdemo/Data/");

    SimpleXMLParser<PointType> simple_parser;
    SimpleXMLParser<PointType>::RoomData roomData;

    std::vector<Entities> allSweeps = summary_parser.getRooms();

    for (size_t i=0; i<allSweeps.size(); i++)
    {
        cout<<"Parsing "<<allSweeps[i].roomXmlFile<<endl;

        roomData = simple_parser.loadRoomFromXML(allSweeps[i].roomXmlFile);
        cout<<"Complete cloud size "<<roomData.completeRoomCloud->points.size()<<endl;
        cout<<"Room waypoint id "<<roomData.roomWaypointId<<std::endl;

        for (size_t i=0; i<roomData.vIntermediateRoomClouds.size(); i++)
        {
            cout<<"Intermediate cloud size "<<roomData.vIntermediateRoomClouds[i]->points.size()<<endl;
            cout<<"Fx: "<<roomData.vIntermediateRoomCloudCamParams[i].fx()<<" Fy: "<<roomData.vIntermediateRoomCloudCamParams[i].fy()<<endl;
        }
    }
}
