#include <dynamic_object_retrieval/summary_iterators.h>

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
        cout << "Please supply the path containing the sweeps..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);

    dynamic_object_retrieval::convex_feature_cloud_map segment_features(data_path);
    dynamic_object_retrieval::convex_keypoint_cloud_map segment_keypoints(data_path);

    for (auto tup : dynamic_object_retrieval::zip(segment_features, segment_keypoints)) {
        HistCloudT::Ptr features;
        CloudT::Ptr keypoints;

        tie(features, keypoints) = tup;

        assert(features->size() == keypoints->size());
    }

    cout << "Convex segment features & keypoints match up!" << endl;

    dynamic_object_retrieval::subsegment_feature_cloud_map subsegment_features(data_path);
    dynamic_object_retrieval::subsegment_keypoint_cloud_map subsegment_keypoints(data_path);

    for (auto tup : dynamic_object_retrieval::zip(subsegment_features, subsegment_keypoints)) {
        HistCloudT::Ptr features;
        CloudT::Ptr keypoints;

        tie(features, keypoints) = tup;

        assert(features->size() == keypoints->size());
    }

    cout << "Subsegment features & keypoints match up!" << endl;

    return 0;
}
