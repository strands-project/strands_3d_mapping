#ifndef Frame_H_
#define Frame_H_

#include <vector>

#include "Camera.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.hpp"
#include "opencv/highgui.h"

#include <Eigen/Dense>

class Camera;

class Frame
{
	public:
	Camera * camera;
	
	int id;
	int framepos;
	
	int featuretype;
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	std::vector<float> keypoint_depth;

	std::vector<Eigen::Vector3f> keypoint_location;

	Frame(Camera * camera_, float * rgb_data_, float * depth_data_);
	~Frame();

	void recalculateFullPoints();
};
#endif
