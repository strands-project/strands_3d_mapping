#include <metaroom_xml_parser/simple_summary_parser.h>
#include <metaroom_xml_parser/simple_xml_parser.h>
#include <metaroom_xml_parser/simple_dynamic_object_parser.h>
#include <metaroom_xml_parser/load_utilities.h>

typedef pcl::PointXYZRGB PointType;
typedef semantic_map_load_utilties::DynamicObjectData<PointType> ObjectData;

typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

using namespace std;

int main(int argc, char** argv)
{
    string folder;

    if (argc > 1){
        folder = argv[1];
    } else {
        cout<<"Please specify the folder from where to load the observations from"<<endl;
        return -1;
    }

    cout<<"Looking for observations..."<<endl;
    vector<string> observation_xmls = semantic_map_load_utilties::getSweepXmls<PointType>(folder);
    cout<<observation_xmls.size()<<" observations found."<<endl;

    for (string observation : observation_xmls){
        cout<<"Observation "<<observation<<endl;
        int slash_pos = observation.find_last_of("/");
        std::string observation_folder = observation.substr(0, slash_pos) + "/";
        // find all object xmls
        QStringList objectFiles = QDir(observation_folder.c_str()).entryList(QStringList("*object*.xml"));

        // load all objects in this observation
        vector<ObjectData> objects = semantic_map_load_utilties::loadAllDynamicObjectsFromSingleSweep<PointType>(observation);

        for (size_t k=0; k<objects.size(); k++){
            auto object = objects[k];
            if(object.vAdditionalViews.size()){
                cout<<observation_folder+ "/" + objectFiles[k].toStdString()<<endl;
            }
        }
    }

    return 1;
}
