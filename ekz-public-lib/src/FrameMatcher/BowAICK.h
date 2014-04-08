#ifndef BowAICK_H_
#define BowAICK_H_
//OpenCV
//#include "cv.h"
//#include "highgui.h"
//#include <opencv.hpp>

#include <string>
#include <iostream>
#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/thread/thread.hpp>

#include "FrameMatcher.h"

using namespace std;

class BowAICK: public FrameMatcher
{
	public:
		int nr_iter;
		float feature_scale;
		float distance_threshold;
		float feature_threshold;
		float bow_threshold;
		float shrinking;
		float stabilety_threshold;

		float converge;
		bool iteration_shrinking;
		bool fitness_shrinking;
		float fitness_constant;

		int max_points;
		BowAICK();
		BowAICK(int max_points_);
		BowAICK(int max_points_, int nr_iter_, float shrinking_,float bow_threshold_, float distance_threshold_,float feature_threshold_);
		~BowAICK();
		Transformation * getTransformation(RGBDFrame * src, RGBDFrame * dst);
		float getAlpha(int iteration);
		float getAlpha(float avg_d2, int iteration);
};

#endif
