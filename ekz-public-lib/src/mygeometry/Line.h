#ifndef Line_H_
#define Line_H_
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include "Point.h"
#include "Plane.h"
#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <Eigen/LU>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <Eigen/Core>

using namespace Eigen;

class Line
{
	public:
	int id;
	float dir_x;
	float dir_y;
	float dir_z;
	float point_x;
	float point_y;
	float point_z;

	float dir_length;
	Eigen::Vector3f dir;
	Eigen::Vector3f point_a;
	Eigen::Vector3f point_b;
	
	Line(Vector3f & p1, Vector3f & p2);
	Line(float x1, float y1, float z1, float x2, float y2, float z2);
	Line(Point * a, Point * b);
	Line(Plane * a, Plane * b);
	Line();
	~Line();
	
	void init(Vector3f & p1, Vector3f & p2);
	void init(float x1, float y1, float z1, float x2, float y2, float z2);

	float distance(Point * point);
	float distance(Vector3f & point);
	float distance(float x, float y, float z);

	void distance(float & de, float & t1, float & t2, Line * line);

	void closestPoints(Vector3f & src_point,Vector3f & dst_point,float & s ,float & t, Line * line);

	void closestPoint(Vector3f & point, float & t,  Vector3f & Q);
};
#endif
