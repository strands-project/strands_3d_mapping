#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "dynamic_object_retrieval/definitions.h"
#include "iss/iss_conf.h"
#include "iss/iss_conf.hpp"
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d_omp.h>
#include <dynamic_object_retrieval/visualize.h>
#include "dynamic_object_retrieval/surfel_type.h"
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include "density_learning/surfel_features.h"

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using NormalT = pcl::Normal;
using NormalCloudT = pcl::PointCloud<NormalT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply the path containing the sweeps..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);

    dynamic_object_retrieval::convex_segment_cloud_map segment_clouds(data_path);
    dynamic_object_retrieval::convex_feature_map segment_features(data_path);
    dynamic_object_retrieval::convex_keypoint_map segment_keypoints(data_path);
    dynamic_object_retrieval::convex_segment_sweep_path_map segment_sweep_paths(data_path);

    boost::filesystem::path last_sweep_path;
    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
    for (auto tup : dynamic_object_retrieval::zip(segment_clouds, segment_features,
                                                  segment_keypoints, segment_sweep_paths)) {
        CloudT::Ptr segment;
        boost::filesystem::path feature_path;
        boost::filesystem::path keypoint_path;
        boost::filesystem::path sweep_path;
        tie(segment, feature_path, keypoint_path, sweep_path) = tup;

        if (sweep_path != last_sweep_path) {
            boost::filesystem::path surfels_path = sweep_path / "surfel_map.pcd";
            surfel_cloud->clear();
            pcl::io::loadPCDFile(surfels_path.string(), *surfel_cloud);
        }

        NormalCloudT::Ptr normals = surfel_features::compute_surfel_normals(surfel_cloud, segment);

        HistCloudT::Ptr features(new HistCloudT);
        CloudT::Ptr keypoints(new CloudT);
        surfel_features::compute_features(features, keypoints, segment, normals, false);

        if (features->empty()) {
            HistT sp;
            for (int i = 0; i < N; ++i) {
                sp.histogram[i] = std::numeric_limits<float>::infinity();
            }
            features->push_back(sp);
            PointT p;
            p.x = p.y = p.z = std::numeric_limits<float>::infinity();
            keypoints->push_back(p);
        }

        feature_path = feature_path.parent_path() / (string("density_") + feature_path.filename().string());
        keypoint_path = keypoint_path.parent_path() / (string("density_") + keypoint_path.filename().string());

        cout << "Found cloud of size: " << segment->size() << endl;
        cout << "Found feature path: " << feature_path.string() << endl;

        pcl::io::savePCDFileBinary(feature_path.string(), *features);
        pcl::io::savePCDFileBinary(keypoint_path.string(), *keypoints);
    }

    return 0;
}
