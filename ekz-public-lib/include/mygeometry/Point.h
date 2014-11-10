#ifndef Point_H_
#define Point_H_
#include <Eigen/Core>
class KeyPoint;
class Point
{

	public:
	KeyPoint * keypoint;
	float x;
	float y;
	float z;
	float w;
	float h;
	Eigen::Vector3f pos;
	Point(float x_, float y_, float z_, float w_,float h_);
	Point();
	~Point();
	void print();
	float distance(Point * point);
};
#endif
