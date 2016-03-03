#ifndef myhull_H
#define myhull_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>

#include <boost/shared_ptr.hpp>
#include <pcl/Vertices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace reglib
{

	class myhull{
		public:

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull;
		std::vector< pcl::Vertices > polygons;

		myhull();
		~myhull();

		void compute(std::vector< Eigen::VectorXd > & input, double alpha = 0.1);
		bool isInlier( Eigen::VectorXd input);
	};

}

#endif // myhull_H
