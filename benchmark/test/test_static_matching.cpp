#include <dynamic_object_retrieval/dynamic_retrieval.h>

#include <object_3d_benchmark/benchmark_retrieval.h>
#include <object_3d_benchmark/benchmark_result.h>
#include <object_3d_benchmark/benchmark_visualization.h>
#include <object_3d_benchmark/benchmark_overlap.h>

#include <tf_conversions/tf_eigen.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using LabelT = semantic_map_load_utilties::LabelledData<PointT>;

vector<pair<CloudT::Ptr, boost::filesystem::path> > get_static_instances(CloudT::Ptr query_cloud, const boost::filesystem::path& data_path)
{
    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    for (const string& xml : folder_xmls) {
        boost::filesystem::path xml_path = boost::filesystem::path(xml);
        // get all convex segments for this sweep
        dynamic_object_retrieval::sweep_convex_segment_cloud_map segments(xml_path.parent_path());

        for (CloudT::Ptr& c : segments) {
            // here it seems like we're gonna have to wrap our own solution by wrapping two octrees (or voxelgrids?)
            double overlap_fraction = benchmark_retrieval::compute_overlap(query_cloud, c);

            if (overlap_fraction > 0.2) {
                CloudT::Ptr visualization_cloud(new CloudT);
                *visualization_cloud += *query_cloud;
                *visualization_cloud += *c;
                cout << "overlap fraction: " << overlap_fraction << endl;
                dynamic_object_retrieval::visualize(visualization_cloud);
            }



        }
    }

    vector<pair<CloudT::Ptr, boost::filesystem::path> > rtn;
    return rtn;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide the path to the data..." << endl;
        exit(0);
    }

    boost::filesystem::path data_path(argv[1]);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    for (const string& sweep_xml : folder_xmls) {
        LabelT labels = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointT>(sweep_xml);

        for (CloudT::Ptr& c : labels.objectClouds) {
            vector<pair<CloudT::Ptr, boost::filesystem::path> > matches = get_static_instances(c, data_path);
        }
    }

    return 0;
}
