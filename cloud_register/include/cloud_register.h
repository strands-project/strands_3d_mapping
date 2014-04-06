#ifndef __CLOUD_REGISTER__
#define __CLOUD_REGISTER__

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <pcl::PointNormal>
{
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};


template <class PointType>
class CloudRegister {
public:

    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointNormal PointNormalT;
    typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

    CloudRegister()
    {

    }

    ~CloudRegister()
    {

    }


   ////////////////////////////////////////////////////////////////////////////////
    /** \brief Align a pair of PointCloud datasets and return the result
      * \param cloud_src the source PointCloud
      * \param cloud_tgt the target PointCloud
      * \param output the resultant aligned source PointCloud
      * \param final_transform the resultant transform between source and target
      */
   static void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
    {
      //
      // Downsample for consistency and speed
      // \note enable this for large datasets
      PointCloud::Ptr src (new PointCloud);
      PointCloud::Ptr tgt (new PointCloud);
      pcl::VoxelGrid<PointT> grid;
      if (downsample)
      {
        grid.setLeafSize (0.05, 0.05, 0.05);
        grid.setInputCloud (cloud_src);
        grid.filter (*src);

        grid.setInputCloud (cloud_tgt);
        grid.filter (*tgt);
      }
      else
      {
        src = cloud_src;
        tgt = cloud_tgt;
      }


      // Compute surface normals and curvature
      PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
      PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

      pcl::NormalEstimation<PointT, PointNormalT> norm_est;
      pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
      norm_est.setSearchMethod (tree);
      norm_est.setKSearch (30);

      norm_est.setInputCloud (src);
      norm_est.compute (*points_with_normals_src);
      pcl::copyPointCloud (*src, *points_with_normals_src);

      norm_est.setInputCloud (tgt);
      norm_est.compute (*points_with_normals_tgt);
      pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

      //
      // Instantiate our custom point representation (defined above) ...
      MyPointRepresentation point_representation;
      // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
      float alpha[4] = {1.0, 1.0, 1.0, 1.0};
      point_representation.setRescaleValues (alpha);

      //
      // Align
      pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
      reg.setTransformationEpsilon (1e-6);
      // Set the maximum distance between two correspondences (src<->tgt) to 10cm
      // Note: adjust this based on the size of your datasets
      reg.setMaxCorrespondenceDistance (0.1);
      // Set the point representation
      reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

      reg.setInputCloud (points_with_normals_src);
      reg.setInputTarget (points_with_normals_tgt);



      //
      // Run the same optimization in a loop and visualize the results
      Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
      PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
      reg.setMaximumIterations (2);
      for (int i = 0; i < 30; ++i)
      {
        PCL_INFO ("Iteration Nr. %d.\n", i);

        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputCloud (points_with_normals_src);
        reg.align (*reg_result);

            //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation () * Ti;

            //if the difference between this transformation and the previous one
            //is smaller than the threshold, refine the process by reducing
            //the maximal correspondence distance
        if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
          reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

        prev = reg.getLastIncrementalTransformation ();

        // visualize current state
//        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
      }

        //
      // Get the transformation from target to source
      targetToSource = Ti.inverse();

      //
      // Transform target back in source frame
      pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

//      p->removePointCloud ("source");
//      p->removePointCloud ("target");

//      PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
//      PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
//      p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
//      p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

//        PCL_INFO ("Press q to continue the registration.\n");
//      p->spin ();

//      p->removePointCloud ("source");
//      p->removePointCloud ("target");

      //add the source to the transformed target
      *output += *cloud_src;

      final_transform = targetToSource;
     }


};

#endif
