#ifndef Matcher_H
#define Matcher_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>

#include <Eigen/Dense>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ICP.h"

namespace matchlib
{
	class Matcher{
		public:
//		CloudData * dst;
		Matcher();
		~Matcher();
//		void setDst(CloudData * dst_);
//		virtual Eigen::MatrixXd getTransform(Eigen::MatrixXd guess);
	};
}

#endif // Matcher_H
