#include "simpleXMLparser.h"

typedef pcl::PointXYZRGB PointType;

using namespace std;

int main(int argc, char** argv)
{
    SimpleXMLParser<PointType> parser;

    SimpleXMLParser<PointType>::RoomData roomData;

    roomData = parser.loadRoomFromXML("SOME_PATH/room_0/room.xml");

    cout<<"Complete cloud size "<<roomData.completeRoomCloud->points.size()<<endl;

    for (size_t i=0; i<roomData.vIntermediateRoomClouds.size(); i++)
    {
        cout<<"Intermediate cloud size "<<roomData.vIntermediateRoomClouds[i]->points.size()<<endl;
        cout<<"Fx: "<<roomData.vIntermediateRoomCloudCamParams[i].fx()<<" Fy: "<<roomData.vIntermediateRoomCloudCamParams[i].fy()<<endl;


    }
}
