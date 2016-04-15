#include "dynamic_object_retrieval/summary_iterators.h"
#include <metaroom_xml_parser/load_utilities.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/visualization/pcl_visualizer.h>

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

using namespace std;

int main(int argc, char** argv)
{
    if (argc < 3) {
        cout << "Please provide the path to the sweep and the indices to add..." << endl;
        return 0;
    }
    boost::filesystem::path sweep_path = boost::filesystem::path(argv[1]);
    vector<int> indices;
    for (int i = 2; i < argc; ++i) {
        indices.push_back(atoi(argv[i]));
    }


    semantic_map_load_utilties::IntermediateCloudCompleteData<PointT> data = semantic_map_load_utilties::loadIntermediateCloudsCompleteDataFromSingleSweep<PointT>(sweep_path.string());

    CloudT::Ptr sweep_cloud(new CloudT);
    for (int i : indices) {
        Eigen::Affine3d e;
        tf::transformTFToEigen(data.vIntermediateRoomCloudTransformsRegistered[i], e);
        Eigen::Matrix4f T = e.matrix().cast<float>();
        for (PointT p : data.vIntermediateRoomClouds[i]->points) {
            if (!pcl::isFinite(p) || p.z < 0.1f) {
                continue;
            }
            p.getVector3fMap() = 2.0f/p.z * p.getVector3fMap();
            p.getVector4fMap() = T*p.getVector4fMap();
            sweep_cloud->push_back(p);
        }
    }

    cout << "Cloud size: " << sweep_cloud->size() << endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(sweep_cloud);
    viewer->addPointCloud<PointT>(sweep_cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }

    boost::filesystem::path sweep_cloud_path = sweep_path.parent_path() / "sweep_illustration_cloud.pcd";
    pcl::io::savePCDFileBinary(sweep_cloud_path.string(), *sweep_cloud);

    return 0;
}

