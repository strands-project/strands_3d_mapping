#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>

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

map<size_t, size_t> load_convex_segment_indices(const boost::filesystem::path& sweep_path)
{
    map<size_t, size_t> convex_segment_indices;
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

    // DEBUG
    dynamic_object_retrieval::convex_segment_cloud_map convex_clouds(data_path);
    // /DEBUG

    map<size_t, size_t> convex_segment_indices;
    vector<CloudT::Ptr> supervoxels;
    for (auto tup : dynamic_object_retrieval::zip(convex_features, convex_keypoints, sweep_paths, indices, convex_clouds)) {
        HistCloudT::Ptr features;
        CloudT::Ptr keypoints;
        boost::filesystem::path sweep_path;
        size_t convex_index;
        CloudT::Ptr cloud;
        tie(features, keypoints, sweep_path, convex_index, cloud) = tup;

        if (convex_index == 0) {
            convex_segment_indices.clear();
            supervoxels.clear();
            convex_segment_indices = load_convex_segment_indices(sweep_path);
            dynamic_object_retrieval::sweep_subsegment_cloud_map sweep_supervoxels(sweep_path);
            for (CloudT::Ptr& s : sweep_supervoxels) {
                supervoxels.push_back(CloudT::Ptr(new CloudT(*s)));
                //supervoxels.push_back(s);
                //supervoxels.push_back(CloudT::Ptr(new CloudT));
                //*supervoxels.back() += *s;
            }
            cout << "New sweep!" << endl;
        }

        cout << "New convex segment!" << endl;
        cout << "Size of convex keypoints: " << keypoints->size() << endl;

        // now we need to iterate through the supervoxels of one sweep and get the keypoint intersection
        for (const pair<size_t, size_t>& ind : convex_segment_indices) {

            // so the problem is that there is garbage attached to ind.second == 0
            // i.e. to the 0:th supervoxel. It seems there are valid points also?
            // Or are they attached to another convex segment (the same) as well?
            /*if (ind.second == 0) {
                continue;
            }*/

            cout << ind.first << " belongs to convex segment " << ind.second << endl;

            if (ind.second == convex_index) {

                //dynamic_object_retrieval::visualize(supervoxels[ind.first]);

                HistCloudT::Ptr supervoxel_features(new HistCloudT);
                CloudT::Ptr supervoxel_keypoints(new CloudT);

                std::stringstream ss;
                ss << std::setw(4) << std::setfill('0') << ind.first;

                boost::filesystem::path subsegment_path = sweep_path / "subsegments";
                boost::filesystem::path feature_path = subsegment_path / (string("feature") + ss.str() + ".pcd");
                boost::filesystem::path keypoint_path = subsegment_path / (string("keypoint") + ss.str() + ".pcd");

                cout << "Size of supervoxel cloud: " << supervoxels[ind.first]->size() << endl;
                cout << "Size of convex keypoints: " << keypoints->size() << endl;
                cout << "Sweep path: " << sweep_path.string() << endl;

                // probably use a kd tree or an octree for this
                pcl::KdTreeFLANN<PointT> kdtree;
                if (supervoxels[ind.first]->size() == 1 && !pcl::isFinite(supervoxels[ind.first]->at(0))) {
                    supervoxel_keypoints->push_back(supervoxels[ind.first]->at(0));
                    HistT h;
                    for (float& f : h.histogram) {
                        f = std::numeric_limits<float>::infinity();
                    }

                    pcl::io::savePCDFileBinary(feature_path.string(), *supervoxel_features);
                    pcl::io::savePCDFileBinary(keypoint_path.string(), *supervoxel_keypoints);

                    continue;
                }
                kdtree.setInputCloud(supervoxels[ind.first]);
                size_t feature_ind = 0;
                for (const PointT& p : keypoints->points) {
                    if (!pcl::isFinite(p)) {
                        ++feature_ind;
                        continue;
                    }
                    vector<int> indices(1);
                    vector<float> distances(1);
                    kdtree.nearestKSearchT(p, 1, indices, distances);
                    if (distances.empty()) {
                        cout << "Distances empty, wtf??" << endl;
                        exit(0);
                    }
                    //cout << "Distance: " << distances[0] << endl;
                    if (sqrt(distances[0]) < 0.05) {
                        supervoxel_features->push_back(features->at(feature_ind));
                        supervoxel_keypoints->push_back(p);
                    }
                    ++feature_ind;
                }

                // save the resulting features and keypoints
                cout << "Size of keypoints: " << supervoxel_keypoints->size() << endl;
                cout << "Size of features: " << supervoxel_features->size() << endl;
                cout << "Keypoint path: " << keypoint_path.string() << endl;
                cout << "Feature path: " << feature_path.string() << endl;

                if (supervoxel_keypoints->empty()) {
                    for (PointT p : supervoxels[ind.first]->points) {
                        p.r = 255;
                        p.g = 0;
                        p.b = 0;
                        cloud->push_back(p);
                        HistT h;
                        for (float& f : h.histogram) {
                            f = std::numeric_limits<float>::infinity();
                        }
                        supervoxel_features->push_back(h);
                        p.x = p.y = p.z = std::numeric_limits<float>::infinity();
                        supervoxel_keypoints->push_back(p);
                    }
                    dynamic_object_retrieval::visualize(cloud);
                }

                pcl::io::savePCDFileBinary(feature_path.string(), *supervoxel_features);
                pcl::io::savePCDFileBinary(keypoint_path.string(), *supervoxel_keypoints);
            }
        }
    }

    return 0;
}
