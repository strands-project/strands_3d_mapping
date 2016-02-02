#ifndef MODEL2_H
#define MODEL2_H

#include <iostream>
#include <Eigen/Dense>
#include "simple_xml_parser.h"
#include "simple_summary_parser.h"
#include <tf_conversions/tf_eigen.h>
#include "tf_conversions/tf_eigen.h"
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "inputframe.h"
//#include "point.h"

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "DistanceWeightFunction2.h"
#include "DistanceWeightFunction.h"
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/integral_image_normal.h>
#include <limits>

#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>

namespace modellib{

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

class Model2
{
	public:
	unsigned int id;

	std::vector< InputFrame * > 		frames;
	std::vector< Eigen::Matrix4f >		poses;
	std::vector< Point * >				fused_points;
    std::vector< std::vector< Point * > >	nonfused_points;

	double maxd;
	double smoothing;
	unsigned int histogram_size;
	double * histogram;

	//Model2(Model2 * src, Model2 * dst, Eigen::Matrix4f pose);
	Model2(std::vector< InputFrame * > & frames_, std::vector< Eigen::Matrix4f > & poses_);
	Model2();
	~Model2();

	void add(InputFrame * frame, Eigen::Matrix4f pose);
	//void renderFrame(InputFrame * frame, Eigen::Matrix4f pose);
	void pruneFromFrame(InputFrame * frame, Eigen::Matrix4f pose);
	//void getInds(unsigned int * inds, InputFrame * frame1, InputFrame * frame2);
    void getInds(unsigned int * inds, std::vector<float> & distances);
	void buildHist(unsigned int * inds, unsigned int nr_pixels, double * hist,  unsigned int nr_bins);
	void getModel(double & mul, double & mean, double & stdval, double * hist,  unsigned int nr_bins);
	void getProbs(double & mul, double & mean, double & stdval, double * probs, double * hist,  unsigned int nr_bins);
	void merge( Model2 * m, Eigen::Matrix4f & pose);

	//void transform(Eigen::Matrix4f pose);
	void align(bool show);
	Eigen::Matrix4f align( Model2 * m, Eigen::Matrix4f pose, bool show);
	Eigen::Matrix4f align( std::vector<InputFrame * > & frames_, std::vector< Eigen::Matrix4f > & poses_, Eigen::Matrix4f pose, bool show);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getFusedCloud();
	
};

}

#endif // MODEL2_H
