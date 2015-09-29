#include "object_3d_retrieval/supervoxel_segmentation.h"
#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <metaroom_xml_parser/load_utilities.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using Graph = supervoxel_segmentation::Graph;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

using namespace dynamic_object_retrieval;

pair<int, vector<string> > convex_segment_cloud(int counter, const boost::filesystem::path& xml_path)
{
    boost::filesystem::path convex_path = xml_path.parent_path() / "convex_segments";

    sweep_summary summary;
    summary.load(convex_path);
    vector<string> segment_paths;

    if (summary.nbr_segments > 0) {
        cout << "Segments already extracted for " << xml_path.string() << ", skipping folder..." << endl;
        counter += summary.nbr_segments;
        convex_segment_map convex_segment_paths(xml_path.parent_path());
        for (const boost::filesystem::path& path : convex_segment_paths) {
            segment_paths.push_back(path.string());
        }
        return make_pair(counter, segment_paths);
    }

    CloudT::Ptr cloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointT>(xml_path.string());

    pcl::ApproximateVoxelGrid<PointT> vf;
    vf.setInputCloud(cloud);
    vf.setLeafSize(0.015f, 0.015f, 0.015f);

    CloudT::Ptr filtered_cloud(new CloudT);
    vf.filter(*filtered_cloud);

    supervoxel_segmentation ss;
    vector<CloudT::Ptr> convex_segments;
    Graph* g = ss.compute_convex_oversegmentation(convex_segments, filtered_cloud, false);

    delete g;

    summary.nbr_segments = convex_segments.size();
    summary.segment_indices.clear(); // could also just be an offset instead of all the indices

    int i = 0;
    for (CloudT::Ptr& c : convex_segments) {
        std::stringstream ss;
        ss << "segment" << std::setw(4) << std::setfill('0') << i;
        boost::filesystem::path segment_path = convex_path / (ss.str() + ".pcd");
        pcl::io::savePCDFileBinary(segment_path.string(), *c);
        segment_paths.push_back(segment_path.string());
        summary.segment_indices.push_back(counter);
        ++i;
        ++counter;
    }

    summary.save(convex_path);

    return make_pair(counter, segment_paths);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the sweeps..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);

    vector<string> folder_xmls = semantic_map_load_utilties::getSweepXmls<PointT>(data_path.string(), true);

    data_summary summary;
    summary.load(data_path);

    summary.index_convex_segment_paths.clear();
    int counter = 0;
    for (const string& xml : folder_xmls) {
        vector<string> segment_paths;
        tie(counter, segment_paths) = convex_segment_cloud(counter, boost::filesystem::path(xml));
        summary.index_convex_segment_paths.insert(summary.index_convex_segment_paths.end(), segment_paths.begin(), segment_paths.end());
        summary.nbr_convex_segments = counter;
    }

    summary.save(data_path);

    return 0;
}
