#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "object_3d_retrieval/pfhrgb_estimation.h"

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<250>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply a point cloud .pcd to segment..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);

    dynamic_object_retrieval::convex_segment_cloud_map segment_clouds(data_path);
    dynamic_object_retrieval::convex_feature_map segment_features(data_path);
    dynamic_object_retrieval::convex_keypoint_map segment_keypoints(data_path);

    for (auto tup : dynamic_object_retrieval::zip(segment_clouds, segment_features, segment_keypoints)) {
        CloudT::Ptr segment;
        boost::filesystem::path feature_path;
        boost::filesystem::path keypoint_path;
        tie(segment, feature_path, keypoint_path) = tup;
        cout << "Found cloud of size: " << segment->size() << endl;
        cout << "Found feature path: " << feature_path.string() << endl;

        HistCloudT::Ptr desc_cloud(new HistCloudT);
        CloudT::Ptr kp_cloud(new CloudT);
        pfhrgb_estimation::compute_features(desc_cloud, kp_cloud, segment, false);

        if (desc_cloud->empty()) {
            // push back one inf point on descriptors and keypoints
            HistT sp;
            for (int i = 0; i < 250; ++i) {
                sp.histogram[i] = std::numeric_limits<float>::infinity();
            }
            desc_cloud->push_back(sp);
            PointT p;
            p.x = p.y = p.z = std::numeric_limits<float>::infinity();
            kp_cloud->push_back(p);
        }

        pcl::io::savePCDFileBinary(feature_path.string(), *desc_cloud);
        pcl::io::savePCDFileBinary(keypoint_path.string(), *kp_cloud);
    }

    return 0;
}
