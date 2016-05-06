#ifndef reglibModelMask_H
#define reglibModelMask_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>

#include <Eigen/Dense>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "../core/RGBDFrame.h"

namespace reglib
{
	class ModelMask{
		public:
		int id;
		cv::Mat mask;
		int width;
		int height;
		bool * maskvec;
		std::vector<int> testw;
		std::vector<int> testh;
		int sweepid;

		ModelMask(cv::Mat mask_);
		~ModelMask();
		cv::Mat getMask();
	};

}

#endif // reglibModelMask_H
