#include "register_objects.h"
#include <iostream>
#include <pcl/io/pcd_io.h>

using std::cout; using std::endl; using std::string; using std::vector;
using std::pair; using std::tuple; using std::tie; using std::make_pair; using std::make_tuple;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

void test_registration_pair(const std::string& file1, const std::string& file2)
{
    CloudT::Ptr cloud1(new CloudT);
    CloudT::Ptr cloud2(new CloudT);
    if (pcl::io::loadPCDFile<PointT>(file1, *cloud1) == -1) exit(0);
    if (pcl::io::loadPCDFile<PointT>(file2, *cloud2) == -1) exit(0);

    Eigen::Matrix3f K;
    K << 570.342, 0.0, 319.5,
         0.0, 570.342, 239.5,
         0.0, 0.0, 1.0;

    register_objects ro;
    ro.set_input_clouds(cloud1, K, cloud2, K);
    ro.do_registration();

    float score = ro.get_match_score();
    cout << "Match score: " << score << endl;

    Eigen::Matrix4f T;
    ro.get_transformation(T);
}

int main(int argc, char** argv)
{
    // SCREENS:
    test_registration_pair("/home/nbore/Workspace/objectness_score/object_segments/segment51/hd_segment.pcd",
                           "/home/nbore/Workspace/objectness_score/object_segments/segment36/hd_segment.pcd");

    // MUGS:
    test_registration_pair("/home/nbore/Workspace/objectness_score/object_segments/segment34/hd_segment.pcd",
                           "/home/nbore/Workspace/objectness_score/object_segments/segment46/hd_segment.pcd");

    // DRAWERS:
    test_registration_pair("/home/nbore/Workspace/objectness_score/object_segments/segment20/hd_segment.pcd",
                           "/home/nbore/Workspace/objectness_score/object_segments/segment37/hd_segment.pcd");

    // SCREEN & DRAWER
    test_registration_pair("/home/nbore/Workspace/objectness_score/object_segments/segment51/hd_segment.pcd",
                           "/home/nbore/Workspace/objectness_score/object_segments/segment37/hd_segment.pcd");

    return 0;
}
