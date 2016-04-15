#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>

#include <metaroom_xml_parser/load_utilities.h>

#include <pcl/surface/mls.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path to the data..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    for (const string& sweep_xml : folder_xmls) {
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

        for (CloudT::Ptr& c : labels.objectClouds) {
            HistCloudT::Ptr query_features(new HistCloudT);
            CloudT::Ptr keypoints(new CloudT);
            pfhrgb_estimation::compute_regularized_query_features(query_features, keypoints, c);
            for (PointT p : keypoints->points) {
                p.r = 255; p.g = 0; p.b = 0;
                c->push_back(p);
            }
            dynamic_object_retrieval::visualize(c);
        }
    }

    return 0;
}

