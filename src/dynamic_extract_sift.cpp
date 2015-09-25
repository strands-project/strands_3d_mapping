#include <extract_sift/extract_sift.h>
#include <metaroom_xml_parser/load_utilities.h>

#include <boost/filesystem.hpp>

using namespace std;

using PointT = pcl::PointXYZRGB;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the sweeps..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    for (const string& xml : folder_xmls) {
        extract_sift::extract_sift_for_sweep(boost::filesystem::path(xml));
    }

    return 0;
}
