#include <semantic_map/mongodb_interface.h>
#include <semantic_map/room_xml_parser.h>
#include <semantic_map/semantic_map_summary_parser.h>

#include <vector>



typedef pcl::PointXYZRGB PointType;

typedef typename SemanticMapSummaryParser::EntityStruct Entities;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::iterator CloudIterator;

using namespace std;

int main(int argc, char** argv)
{

    if (argc < 2)
    {
        cout<<"Please provide folder where to save the data."<<endl;
        return -1;
    }

    string folderPath = argv[1] + string("/");
    string indexXmlPath = folderPath + string("index.xml");

    map<string, vector<string> > observationMatches;
    SemanticMapSummaryParser summary_parser(indexXmlPath);
    summary_parser.createSummaryXML<PointType>(folderPath);

    SemanticRoomXMLParser<PointType> simple_parser;
    SemanticRoom<PointType> roomData;

    std::vector<Entities> allSweeps = summary_parser.getRooms();

    // initialize ros
    // Set up ROS.
    ros::init(argc, argv, "Add_to_mongo_node");
    ros::NodeHandle n;
    ros::NodeHandle aRosNode("~");

    MongodbInterface* m_MongodbInterface = new MongodbInterface(aRosNode);

    for (size_t i=0; i<allSweeps.size(); i++)
    {
        cout<<"Parsing "<<allSweeps[i].roomXmlFile<<endl;
        SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(allSweeps[i].roomXmlFile,true);
        m_MongodbInterface->logRoomToDB(aRoom,allSweeps[i].roomXmlFile);

    }


}
