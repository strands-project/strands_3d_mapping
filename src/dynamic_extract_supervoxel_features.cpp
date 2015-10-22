#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>

#include <cereal/archives/binary.hpp>
#include <cereal/types/map.hpp>

using namespace std;

// we need to put all of this in a nice library and link properly
using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

map<int, int> load_convex_segment_indices(const boost::filesystem::path& sweep_path)
{
    map<int, int> convex_segment_indices;
    std::ifstream in((sweep_path / "subsegments" / "convex_segment_indices.cereal").string(), ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(convex_segment_indices);
    }
    in.close();
    return convex_segment_indices;
}

int main(int argc, char** argv)
{
    boost::filesystem::path data_path(argv[1]);

    dynamic_object_retrieval::convex_feature_cloud_map convex_features(data_path);
    dynamic_object_retrieval::convex_keypoint_cloud_map convex_keypoints(data_path);
    dynamic_object_retrieval::convex_segment_sweep_path_map sweep_paths(data_path);
    dynamic_object_retrieval::convex_segment_index_map indices(data_path);

    map<int, int> convex_segment_indices;
    vector<CloudT::Ptr> supervoxels;
    for (auto tup : dynamic_object_retrieval::zip(convex_features, convex_keypoints, sweep_paths, indices)) {
        HistCloudT::Ptr features;
        CloudT::Ptr keypoints;
        boost::filesystem::path sweep_path;
        size_t convex_index;
        tie(features, keypoints, sweep_path, convex_index) = tup;

        if (convex_index == 0) {
            convex_segment_indices.clear();
            supervoxels.clear();
            convex_segment_indices = load_convex_segment_indices(sweep_path);
            dynamic_object_retrieval::sweep_subsegment_cloud_map sweep_supervoxels(sweep_path);
            for (CloudT::Ptr& s : sweep_supervoxels) {
                supervoxels.push_back(s);
            }
        }

        // now we need to iterate through the supervoxels of one sweep and get the keypoint intersection
        for (const pair<int, int>& p : convex_segment_indices) {
            if (p.second == convex_index) {

            }
        }
    }

    return 0;
}
