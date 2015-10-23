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
    if (argc < 2) {
        cout << "Please supply the path containing the sweeps..." << endl;
        return -1;
    }

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
        for (const pair<int, int>& ind : convex_segment_indices) {
            if (ind.second == convex_index) {
                HistCloudT::Ptr supervoxel_features(new HistCloudT);
                CloudT::Ptr supervoxel_keypoints(new CloudT);

                // probably use a kd tree or an octree for this
                pcl::KdTreeFLANN<PointT> kdtree;
                kdtree.setInputCloud(supervoxels[ind.first]);
                size_t feature_ind = 0;
                for (const PointT& p : keypoints->points) {
                    vector<int> indices(1);
                    vector<float> distances(1);
                    kdtree.nearestKSearchT(p, 1, indices, distances);
                    if (distances.empty() || distances.empty()) {
                        cout << "Distances empty, wtf??" << endl;
                        exit(0);
                    }
                    if (sqrt(distances[0]) < 0.005) {
                        supervoxel_features->push_back(features->at(feature_ind));
                        supervoxel_keypoints->push_back(p);
                    }
                    ++feature_ind;
                }

                // save the resulting features and keypoints
                std::stringstream ss;
                ss << std::setw(4) << std::setfill('0') << ind.first;

                boost::filesystem::path subsegment_path = sweep_path / "subsegments";
                boost::filesystem::path feature_path = subsegment_path / (string("feature") + ss.str() + ".pcd");
                boost::filesystem::path keypoint_path = subsegment_path / (string("keypoint") + ss.str() + ".pcd");

                pcl::io::savePCDFileBinary(feature_path.string(), *supervoxel_features);
                pcl::io::savePCDFileBinary(keypoint_path.string(), *supervoxel_keypoints);
            }
        }
    }

    return 0;
}
