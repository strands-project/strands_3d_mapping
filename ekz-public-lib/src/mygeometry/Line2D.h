#ifndef Line2D_H_
#define Line2D_H_
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include "Point.h"
#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <Eigen/LU>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/Core>
#include "HasDistance.h"

using namespace Eigen;
using namespace std;

class Line2DChain;
class Line2D : HasDistance
{
	public:
	int id;
	float weight;

	float normal_w;
	float normal_h;
	float normal2_w;
	float normal2_h;
	float point_w;
	float point_h;
	
	Line2D(vector<float> * p_width, vector<float> * p_height, vector<float> * p_weight);
	~Line2D();

	float distance(float w, float h);
};
#endif
