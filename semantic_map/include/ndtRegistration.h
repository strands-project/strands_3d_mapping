#ifndef __NDT_SEMANTIC_MAP_REGISTRATION__H
#define __NDT_SEMANTIC_MAP_REGISTRATION__H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/io/pcd_io.h>

//#include "ndt.h"
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <string>
#include <vector>


template <class PointType>
class NdtRegistration {
public:
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    NdtRegistration()
    {
    }

    ~NdtRegistration()
    {

    }

    static CloudPtr registerClouds(CloudPtr inputCloud, CloudPtr targetCloud, Eigen::Matrix4f& finalTransform, Eigen::Matrix4f initialTransform = Eigen::Matrix4f::Identity())
    {
        CloudPtr transformedCloud(new Cloud());

        // Filtering input scan to roughly 10% of original size to increase speed of registration.
        CloudPtr filtered_cloud (new Cloud());
        pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
        approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
        approximate_voxel_filter.setInputCloud (inputCloud);
        approximate_voxel_filter.filter (*filtered_cloud);
//        std::cout << "Filtered cloud contains " << filtered_cloud->size () << std::endl;

        // Initializing Normal Distributions Transform (NDT).
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;

        // Setting scale dependent NDT parameters
        // Setting minimum transformation difference for termination condition.
        ndt.setTransformationEpsilon (0.01);
        // Setting maximum step size for More-Thuente line search.
        ndt.setStepSize (0.1);
        //Setting Resolution of NDT grid structure (VoxelGridCovariance).
        ndt.setResolution (1.0);

        // Setting max number of registration iterations.
        ndt.setMaximumIterations (35);

        // Setting point cloud to be aligned.
        ndt.setInputTarget (filtered_cloud);
        // Setting point cloud to be aligned to.
        ndt.setInputTarget (targetCloud);

        // Set initial alignment estimate found using robot odometry.
//        Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ ());
//        Eigen::Translation3f init_translation (0.0, 0.0, 0);
//        Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

        // Calculating required rigid transform to align the input cloud to the target cloud.
        CloudPtr output_cloud (new Cloud());
        ndt.align (*output_cloud, initialTransform);

//        std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
//                  << " score: " << ndt.getFitnessScore () << std::endl;

        // Transforming unfiltered, input cloud using found transform.
        pcl::transformPointCloud (*inputCloud, *transformedCloud, ndt.getFinalTransformation ());

        finalTransform = ndt.getFinalTransformation();

        // Saving transformed input cloud.
//        pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *transformedCloud);

        return transformedCloud;
    }
};



#endif
