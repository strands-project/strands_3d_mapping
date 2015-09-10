#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <tf_conversions/tf_eigen.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
#include <metaroom_xml_parser/simple_summary_parser.h>

using namespace std;

using Entities = SimpleSummaryParser::EntityStruct;
using PointT = pcl::PointXYZRGB;

void get_rooms(vector<Entities>& entities, const string& summary_xml_path)
{
    SimpleSummaryParser summary_parser(summary_xml_path + "index.xml");
    summary_parser.createSummaryXML(summary_xml_path);
    entities = summary_parser.getRooms();
    cout << "Entities: " << entities.size() << endl;
}

void get_room_from_xml(vector<string>& cloud_paths, vector<Entities>& entities)
{
    SimpleXMLParser<PointT> parser;
    vector<string> xml_nodes_to_parse = {"RoomIntermediateCloud", "IntermediatePosition", "RoomStringId"};  // "RoomCompleteCloud", "RoomDynamicClusters"

    for (SimpleSummaryParser::EntityStruct& entity : entities) {
        SimpleXMLParser<PointT>::RoomData room = parser.loadRoomFromXML(entity.roomXmlFile, xml_nodes_to_parse);

        string room_folder = boost::filesystem::path(entity.roomXmlFile).parent_path().string();

        int counter = 0;
        int nbr_found = 0;
        for (const tf::StampedTransform& Tj : room.vIntermediateRoomCloudTransforms) {
            Eigen::Affine3d e;
            tf::transformTFToEigen(Tj, e);
            Eigen::Matrix3d R = e.rotation();
            // only take the scans looking down far enough
            if (acos(Eigen::Vector3d::UnitZ().dot(R.col(2))) < 0.6*M_PI) {
                ++counter;
                continue;
            }


            stringstream ss;
            ss << room_folder << "/intermediate_cloud" << setw(4) << setfill('0') << counter << ".pcd";

            cloud_paths.push_back(ss.str());
            ++counter;
            ++nbr_found;
        }

        cout << "Found " << nbr_found << " scans looking down far enough" << endl;
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the sweeps..." << endl;
        return 0;
    }

    // TODO: Change this path to be configurable, maybe have a global conf file?
    string summary_xml_path(argv[1]); //"/home/nbore/Data/semantic_map";
    vector<Entities> entities;
    get_rooms(entities, summary_xml_path);
    vector<string> cloud_paths;
    get_room_from_xml(cloud_paths, entities);

    string segments_path = summary_xml_path + "/scan_segments";
    boost::filesystem::create_directory(segments_path);

    int counter = 0;
    for (const string& path : cloud_paths) {
        string scan_path = segments_path + "/scan" + to_string(counter);
        boost::filesystem::create_directory(scan_path);
        string metadata_file = scan_path + "/metadata.txt";
        ofstream out;
        out.open(metadata_file);
        out << path << endl;
        ++counter;
    }

    return 0;
}
