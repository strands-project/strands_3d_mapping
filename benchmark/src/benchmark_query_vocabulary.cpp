#include <metaroom_xml_parser/load_utilities.h>
#include <pcl/point_types.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide sweep xml to load..." << endl;
        return -1;
    }

    string sweep_xml(argv[1]);
    LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

    return 0;
}
