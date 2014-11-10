#include "simpleXMLparser.h"

typedef pcl::PointXYZRGB PointType;

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 2)
    {
        cout<<"Please provide XML path as argument."<<endl;
        return -1;
    } else {

    }


    SimpleXMLParser<PointType> parser;

    SimpleXMLParser<PointType>::RoomData roomData;

    roomData = parser.loadRoomFromXML(argv[1]);

    cout<<"Complete cloud size "<<roomData.completeRoomCloud->points.size()<<endl;

    for (size_t i=0; i<roomData.vIntermediateRoomClouds.size(); i++)
    {
        cout<<"Intermediate cloud size "<<roomData.vIntermediateRoomClouds[i]->points.size()<<endl;
        cout<<"Fx: "<<roomData.vIntermediateRoomCloudCamParams[i].fx()<<" Fy: "<<roomData.vIntermediateRoomCloudCamParams[i].fy()<<endl;
    }
}
