#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "ndt.h"
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "ndtRegistration.h"

typedef pcl::PointXYZRGB PointType;

int
main (int argc, char** argv)
{
  // Loading first scan of room.
  pcl::PointCloud<PointType>::Ptr target_cloud (new pcl::PointCloud<PointType>);
  if (pcl::io::loadPCDFile<PointType> ("/home/rares/code/ros_workspace/semantic_mapper_ws/test/difference1_cluster1.pcd", *target_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<PointType>::Ptr input_cloud (new pcl::PointCloud<PointType>);
  if (pcl::io::loadPCDFile<PointType> ("/home/rares/code/ros_workspace/semantic_mapper_ws/test/difference2_cluster1.pcd", *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;

  Eigen::Matrix4f finalTransformation;
  pcl::PointCloud<PointType>::Ptr output_cloud = NdtRegistration<PointType>::registerClouds(input_cloud, target_cloud,finalTransformation);

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<PointType>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<PointType> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<PointType>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<PointType> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0);
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
