#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <metaroom_xml_parser/load_utilities.h>

#include "dynamic_object_retrieval/summary_types.h"

using namespace std;
using namespace dynamic_object_retrieval;

using PointT = pcl::PointXYZRGB;

// TODO: shouldn't this be in a header?
void init_folders(const boost::filesystem::path& data_path)
{
    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    data_summary summary;
    summary.nbr_sweeps = folder_xmls.size();

    summary.save(data_path);

    for (const string& xml : folder_xmls) {
        boost::filesystem::path xml_path(xml);
        boost::filesystem::path convex_path = xml_path.parent_path() / "convex_segments";
        boost::filesystem::path subsegment_path = xml_path.parent_path() / "subsegments";
        boost::filesystem::create_directory(convex_path);
        boost::filesystem::create_directory(subsegment_path);

        sweep_summary convex_summary;
        convex_summary.save(convex_path);
        sweep_summary subsegment_summary;
        subsegment_summary.save(subsegment_path);
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the sweeps..." << endl;
        return 0;
    }

    init_folders(boost::filesystem::path(argv[1]));

    return 0;
}
