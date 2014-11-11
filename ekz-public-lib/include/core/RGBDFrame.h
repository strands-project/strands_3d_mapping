#ifndef RGBDFrame_H_
#define RGBDFrame_H_

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <stdio.h>
#include <string.h>

#include "KeyPointSet.h"
#include "Plane.h"
#include "Point.h"

#include "FeatureExtractor.h"
#include "RGBDSegmentation.h"
#include "FloatHistogramFeatureDescriptor.h"

#include "FrameInput.h"

using namespace std;
class RGBDFrame{
	public:
	int id;
	FrameInput * input;

	FeatureDescriptor * image_descriptor;
	KeyPointSet * keypoints;
	
	Segments * segments;
	
	RGBDFrame();
	RGBDFrame(FrameInput * fi, FeatureExtractor * extractor, RGBDSegmentation * segmenter, bool verbose = false);
	~RGBDFrame();
	
	public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif
