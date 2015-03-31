#ifndef FeatureExtractor_H_
#define FeatureExtractor_H_

//OpenCV
#include "cv.h"
#include "highgui.h"
#include <opencv2/opencv.hpp>
//#include "highgui.h"
#include "KeyPointSet.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/features/integral_image_normal.h>
#include "FrameInput.h"

//bool comparison_f (KeyPoint * i,KeyPoint * j) { return (i->stabilety>j->stabilety); }

class Calibration;
class FeatureExtractor
{
	public:
		bool debugg;
		bool verbose;
		void setVerbose(bool b);
		void setDebugg(bool b);

		virtual KeyPointSet * getKeyPointSet(FrameInput * fi);
		FeatureExtractor();
		virtual ~FeatureExtractor();
};

#include "OrbExtractor.h"
#include "SurfExtractor.h"
#endif
