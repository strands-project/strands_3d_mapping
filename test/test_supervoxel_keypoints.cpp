#include <dynamic_object_retrieval/summary_types.h>
#include <dynamic_object_retrieval/summary_iterators.h>
#include <dynamic_object_retrieval/visualize.h>

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please supply a point cloud .pcd to segment..." << endl;
        return -1;
    }

    int colormap[][3] = {
        {166,206,227},
        {31,120,180},
        {178,223,138},
        {51,160,44},
        {251,154,153},
        {227,26,28},
        {253,191,111},
        {255,127,0},
        {202,178,214},
        {106,61,154},
        {255,255,153},
        {177,89,40},
        {141,211,199},
        {255,255,179},
        {190,186,218},
        {251,128,114},
        {128,177,211},
        {253,180,98},
        {179,222,105},
        {252,205,229},
        {217,217,217},
        {188,128,189},
        {204,235,197},
        {255,237,111}
    };

    boost::filesystem::path data_path(argv[1]);

    dynamic_object_retrieval::subsegment_keypoint_cloud_map subsegment_keypoints(data_path);
    dynamic_object_retrieval::subsegment_index_map sweep_indices(data_path);

    CloudT::Ptr visualization_cloud(new CloudT);
    for (auto tup : dynamic_object_retrieval::zip(subsegment_keypoints, sweep_indices)) {
        CloudT::Ptr keypoints;
        size_t ind;
        tie(keypoints, ind) = tup;
        if (ind == 0) {
            if (!visualization_cloud->empty()) {
                dynamic_object_retrieval::visualize(visualization_cloud);
            }
            visualization_cloud->clear();
        }
        for (PointT p : keypoints->points) {
            p.r = colormap[ind%24][0];
            p.g = colormap[ind%24][1];
            p.b = colormap[ind%24][2];
            visualization_cloud->push_back(p);
        }
    }

    return 0;
}
