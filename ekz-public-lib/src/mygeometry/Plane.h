#ifndef Plane_H_
#define Plane_H_
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

#include "PlaneChain.h"

using namespace Eigen;
using namespace std;

class PlaneChain;
class Plane : HasDistance
{
	public:
	int id;
	float weight;
	Matrix3f plane_points;
	float normal_x;
	float normal_y;
	float normal_z;
	float point_x;
	float point_y;
	float point_z;
	VectorXf S;
	std::vector<int> p_h;
	std::vector<int> p_w;
	float a,b,c,d;
	
	PlaneChain * chain;

	std::vector<float> * x_vec;
	std::vector<float> * y_vec;
	std::vector<float> * z_vec;
	std::vector<int> * w_vec;
	std::vector<int> * h_vec;
	
	//Plane();
	Plane(Point * a, Point * b,Point * c);
	Plane(pcl::PointXYZRGBNormal * pa, pcl::PointXYZRGBNormal * pb,pcl::PointXYZRGBNormal * pc);

	Plane(std::vector<Point *> * points);
	Plane(std::vector<float> * px, std::vector<float> * py, std::vector<float> * pz, std::vector<int> * pw, std::vector<int> * ph);
	Plane(std::vector<pcl::PointXYZRGBNormal *> * points);
	
	Plane();
	~Plane();
	
	void setup(Point * pa, Point * pb,Point * pc);
	void setup( pcl::PointXYZRGBNormal * pa,  pcl::PointXYZRGBNormal * pb, pcl::PointXYZRGBNormal * pc);
	
	void setup(std::vector<Point *> * points);
	void setup(std::vector< pcl::PointXYZRGBNormal *> * points);
	
	void getABCDcoefs();
	
	void project_points(std::vector<Point *> points);
	void project_points(std::vector<pcl::PointXYZRGBNormal *> points);
	
	float distance(float x, float y, float z);
	float proj_distance(float x, float y, float z);
	float distance(Point * point);
	float angle(Plane * p);
	float angle(float nx, float ny, float nz);
	float distance(pcl::PointXYZRGBNormal * point);
	std::vector< Plane *> * extract_subplanes(std::vector<Point *> * points);
	
	void merge(Plane * pl);
};
#endif
