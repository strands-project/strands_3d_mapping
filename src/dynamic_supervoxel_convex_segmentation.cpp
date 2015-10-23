#include "object_3d_retrieval/supervoxel_segmentation.h"
#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"

#include <pcl/io/pcd_io.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <cereal/archives/binary.hpp>
#include <cereal/types/map.hpp>

#include <metaroom_xml_parser/load_utilities.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using Graph = supervoxel_segmentation::Graph;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

using namespace dynamic_object_retrieval;

// now we also need to add the number of sweep that we have traversed
tuple<int, int, vector<string>, vector<string> > supervoxel_convex_segment_cloud(int convex_counter, int supervoxel_counter,
                                                                           const boost::filesystem::path& xml_path)
{
    boost::filesystem::path convex_path = xml_path.parent_path() / "convex_segments";

    sweep_summary convex_summary;
    convex_summary.load(convex_path);
    vector<string> segment_paths;

    boost::filesystem::path supervoxel_path = xml_path.parent_path() / "subsegments";

    sweep_summary supervoxel_summary;
    supervoxel_summary.load(supervoxel_path);
    vector<string> supervoxel_paths;

    if (convex_summary.nbr_segments > 0 && supervoxel_summary.nbr_segments > 0) {
        cout << "Segments already extracted for " << xml_path.string() << ", skipping folder..." << endl;
        convex_counter += convex_summary.nbr_segments;
        supervoxel_counter += supervoxel_summary.nbr_segments;
        convex_segment_map convex_segment_paths(xml_path.parent_path());
        for (const boost::filesystem::path& path : convex_segment_paths) {
            segment_paths.push_back(path.string());
        }
        subsegment_map supervoxel_segment_paths(xml_path.parent_path());
        for (const boost::filesystem::path& path : supervoxel_segment_paths) {
            supervoxel_paths.push_back(path.string());
        }
        return make_tuple(convex_counter, supervoxel_counter, segment_paths, supervoxel_paths);
    }

    CloudT::Ptr cloud = semantic_map_load_utilties::loadMergedCloudFromSingleSweep<PointT>(xml_path.string());

    // the supervoxel segmentation also needs to return the supervoxels
    // and the overal graph among the supervoxels + supervoxel-convex segments association
    supervoxel_segmentation ss;
    Graph* g;
    vector<CloudT::Ptr> supervoxels;
    vector<CloudT::Ptr> convex_segments;
    map<size_t, size_t> indices;
    std::tie(g, supervoxels, convex_segments, indices) = ss.compute_convex_oversegmentation(cloud, false);

    convex_summary.nbr_segments = convex_segments.size();
    convex_summary.segment_indices.clear(); // could also just be an offset instead of all the indices

    int i = 0;
    for (CloudT::Ptr& c : convex_segments) {
        std::stringstream ss;
        ss << "segment" << std::setw(4) << std::setfill('0') << i;
        boost::filesystem::path segment_path = convex_path / (ss.str() + ".pcd");
        pcl::io::savePCDFileBinary(segment_path.string(), *c);
        segment_paths.push_back(segment_path.string());
        convex_summary.segment_indices.push_back(convex_counter);
        ++i;
        ++convex_counter;
    }

    convex_summary.save(convex_path);

    supervoxel_summary.nbr_segments = supervoxels.size();
    supervoxel_summary.segment_indices.clear();

    i = 0;
    for (CloudT::Ptr& c : supervoxels) {
        std::stringstream ss;
        ss << "segment" << std::setw(4) << std::setfill('0') << i;
        boost::filesystem::path segment_path = supervoxel_path / (ss.str() + ".pcd");
        pcl::io::savePCDFileBinary(segment_path.string(), *c);
        supervoxel_paths.push_back(segment_path.string());
        supervoxel_summary.segment_indices.push_back(supervoxel_counter);
        ++i;
        ++supervoxel_counter;
    }

    supervoxel_summary.save(supervoxel_path);

    // finally, also save the graph and the indices in the subsegments folder
    ss.save_graph(*g, (supervoxel_path / "graph.cereal").string());
    ofstream out((supervoxel_path / "convex_segment_indices.cereal").string(), std::ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(indices);
    }
    out.close();

    delete g;

    return make_tuple(convex_counter, supervoxel_counter, segment_paths, supervoxel_paths);
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
    summary.subsegment_type = "supervoxel";

    summary.index_convex_segment_paths.clear();
    int convex_counter = 0;
    int supervoxel_counter = 0;
    for (const string& xml : folder_xmls) {
        vector<string> segment_paths;
        vector<string> supervoxel_paths;
        tie(convex_counter, supervoxel_counter, segment_paths, supervoxel_paths) =
                supervoxel_convex_segment_cloud(convex_counter, supervoxel_counter, boost::filesystem::path(xml));
        summary.index_convex_segment_paths.insert(summary.index_convex_segment_paths.end(), segment_paths.begin(), segment_paths.end());
        summary.nbr_convex_segments = convex_counter;
        summary.nbr_subsegments = supervoxel_counter;
    }

    summary.save(data_path);

    return 0;
}
