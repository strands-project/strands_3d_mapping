#include "dynamic_object_retrieval/summary_types.h"
#include "dynamic_object_retrieval/summary_iterators.h"
#include "dynamic_object_retrieval/definitions.h"
#include <dynamic_object_retrieval/visualize.h>

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
    if (argc < 3) {
        cout << "Please supply the path containing the sweeps..." << endl;
        cout << "And a threshold for the ISS keypoints..." << endl;
        return -1;
    }

    boost::filesystem::path data_path(argv[1]);
    float threshold = atof(argv[2]);

    dynamic_object_retrieval::convex_segment_cloud_map segment_clouds(data_path);
    dynamic_object_retrieval::convex_keypoint_map segment_keypoints(data_path);

    for (auto tup : dynamic_object_retrieval::zip(segment_clouds, segment_keypoints)) {
        CloudT::Ptr segment;
        boost::filesystem::path keypoint_path;
        tie(segment, keypoint_path) = tup;

        CloudT::Ptr keypoints(new CloudT);
        keypoint_path = keypoint_path.parent_path() / (string("density_") + keypoint_path.filename().string());
        pcl::io::loadPCDFile(keypoint_path.string(), *keypoints);

        CloudT::Ptr vis_cloud(new CloudT);
        *vis_cloud += *segment;

        for (PointT p : keypoints->points) {
            cout << "Point threshold is " << p.rgb << endl;
            if (p.rgb < threshold) {
                p.r = 255; p.g = 0; p.b = 0;
                vis_cloud->push_back(p);
            }
        }

        dynamic_object_retrieval::visualize(vis_cloud);
    }

    return 0;
}
