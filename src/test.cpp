#include "k_means_tree/k_means_tree.h"
#include "vocabulary_tree/vocabulary_tree.h"
#include "grouped_vocabulary_tree/grouped_vocabulary_tree.h"
#include "reweighted_vocabulary_tree/reweighted_vocabulary_tree.h"

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/nbore/Workspace/objectness_score/kinect_scenes/scene1006.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    k_means_tree<pcl::PointXYZRGB, 8> kmt3;
    kmt3.set_input_cloud(cloud);
    kmt3.add_points_from_input_cloud();

    std::vector<size_t> temp = kmt3.sample_with_replacement(kmt3.size());
    size_t ind = 54588;//temp[0];
    std::cout << "ind: " << ind << std::endl;
    pcl::PointXYZRGB point = kmt3.get_cloud()->at(ind);

    for (size_t depth = 0; depth < 5; ++depth) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr nodecloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        kmt3.get_cloud_for_point_at_level_optimized(nodecloud, point, depth);
        std::cout << "node cloud size: " << nodecloud->size() << std::endl;
        /*for (pcl::PointXYZRGB& p : nodecloud->points) {
            p.r = 255;
            p.g = 0;
            p.g = 0;
        }*/

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(nodecloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (nodecloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        viewer->addCoordinateSystem (1.0);
        viewer->initCameraParameters ();
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce(100);
        }
    }

    k_means_tree<pcl::Histogram<100>, 8> kmt;
    vocabulary_tree<pcl::Histogram<100>, 8> vt;
    reweighted_vocabulary_tree<pcl::Histogram<100>, 8> rvt;
    grouped_vocabulary_tree<pcl::Histogram<100>, 8> gvt;

    return 0;
}
