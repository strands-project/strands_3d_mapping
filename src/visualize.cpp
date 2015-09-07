#include "dynamic_object_retrieval/visualize.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <cereal/archives/binary.hpp>

POINT_CLOUD_REGISTER_POINT_STRUCT (HistT,
                                   (float[250], histogram, histogram)
)

using namespace std;

namespace dynamic_object_retrieval {

void visualize(CloudT::Ptr& cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer->addPointCloud<PointT>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

void visualize(CloudT::Ptr& cloud, float subsample_size)
{
    CloudT::Ptr subsampled_cloud(new CloudT);
    pcl::ApproximateVoxelGrid<PointT> vf;
    vf.setInputCloud(cloud);
    vf.setLeafSize(subsample_size, subsample_size, subsample_size);
    vf.filter(*subsampled_cloud);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(subsampled_cloud);
    viewer->addPointCloud<PointT>(subsampled_cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}

// we need to put these in some file as they will be used throughout, summary_convenience?
void save_vocabulary(vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path)
{
    ofstream out((vocabulary_path / "vocabulary.cereal").string(), ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(vt);
    }
}

void load_vocabulary(vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path)
{
    ifstream in((vocabulary_path / "vocabulary.cereal").string(), ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(vt);
    }
}

void save_vocabulary(grouped_vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path)
{
    ofstream out((vocabulary_path / "grouped_vocabulary.cereal").string(), ios::binary);
    {
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(vt);
    }

    //string group_file = segment_path + "/" + descriptor_config::grouped_associations_file;
    //vt.save_group_associations(group_file);
}

void load_vocabulary(grouped_vocabulary_tree<HistT, 8>& vt, const boost::filesystem::path& vocabulary_path)
{
    ifstream in((vocabulary_path / "grouped_vocabulary.cereal").string(), ios::binary);
    {
        cereal::BinaryInputArchive archive_i(in);
        archive_i(vt);
    }

    //string group_file = segment_path + "/" + descriptor_config::grouped_associations_file;
    //vt.load_group_associations(group_file);
}

}
