#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>
#include <dynamic_object_retrieval/definitions.h>
#include <object_3d_retrieval/pfhrgb_estimation.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using HistT = pcl::Histogram<N>;
using HistCloudT = pcl::PointCloud<HistT>;

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[N], histogram, histogram)
)

int main(int argc, char** argv)
{
    boost::filesystem::path data_path(argv[1]);

    dynamic_object_retrieval::convex_segment_cloud_map convex_clouds(data_path);
    dynamic_object_retrieval::convex_keypoint_cloud_map convex_keypoints(data_path);
    dynamic_object_retrieval::convex_segment_index_map convex_index(data_path);

    CloudT::Ptr visualization_cloud(new CloudT);
    for (auto tup : dynamic_object_retrieval::zip(convex_clouds, convex_index)) {
        CloudT::Ptr cloud;
        CloudT::Ptr keypoints(new CloudT);
        size_t index;
        tie(cloud, index) = tup;

        HistCloudT::Ptr features(new HistCloudT);
        pfhrgb_estimation::compute_surfel_features(features, keypoints, cloud, false);

        if (index == 0 && !visualization_cloud->empty()) {
            dynamic_object_retrieval::visualize(visualization_cloud);
            visualization_cloud->clear();
        }

        *visualization_cloud += *cloud;

        for (PointT p : keypoints->points) {
            p.r = 255;
            p.g = 0;
            p.b = 0;
            visualization_cloud->push_back(p);
        }

    }

    return 0;
}

