#include "Line.h"
int line_counter = 0;

Line::Line(Vector3f & p1, Vector3f & p2){
	init(p1[0],p1[1],p1[2],p2[0],p2[1],p2[2]);
}

Line::Line(float x1, float y1, float z1, float x2, float y2, float z2){
	init(x1, y1, z1, x2, y2, z2);
}

void Line::init(Vector3f & p1, Vector3f & p2){
	init(p1[0],p1[1],p1[2],p2[0],p2[1],p2[2]);
}

void Line::init(float x1, float y1, float z1, float x2, float y2, float z2){
	id = line_counter++;
	dir[0] = x1-x2;
	dir[1] = y1-y2;
	dir[2] = z1-z2;
	dir_length = dir.norm();
	dir /= dir_length;
	point_a[0] = x1;
	point_a[1] = y1;
	point_a[2] = z1;
	point_b = point_a+dir;
	//point_b[0] = x2;
	//point_b[1] = y2;
	//point_b[2] = z2;

	dir_x = dir[0];
	dir_y = dir[1];
	dir_z = dir[2];
	point_x = x1;
	point_y = y1;
	point_z = z1;
}

Line::Line(Point * pa, Point * pb){
	id = line_counter++;
}

Line::Line(Plane * pa,Plane * pb)
{
	id = line_counter++;
	
	Eigen::Vector3f pa_norm(pa->normal_x,pa->normal_y,pa->normal_z);
	Eigen::Vector3f pa_mean(pa->point_x,pa->point_y,pa->point_z);
	Eigen::Vector3f pb_norm(pb->normal_x,pb->normal_y,pb->normal_z);
	Eigen::Vector3f pb_mean(pb->point_x,pb->point_y,pb->point_z);
	dir = pa_norm.cross(pb_norm);
	dir.normalize();
	Eigen::Vector3f orthLineDir = dir.cross(pa_norm);
	orthLineDir.normalize();
	float d = (pb_mean-pa_mean).dot(pb_norm)/(pb_norm.dot(orthLineDir));
	point_a = pa_mean+d*orthLineDir;
	point_b = point_a+dir;

	//ROS_INFO("norm1(%f,%f,%f) len: %f",pa_norm(0),pa_norm(1),pa_norm(2),pa_norm.norm());
	//ROS_INFO("mean1(%f,%f,%f) len: %f",pa_mean(0),pa_mean(1),pa_mean(2),pa_mean.norm());
	//ROS_INFO("norm2(%f,%f,%f) len: %f",pb_norm(0),pb_norm(1),pb_norm(2),pb_norm.norm());
	//ROS_INFO("mean2(%f,%f,%f) len: %f",pb_mean(0),pb_mean(1),pb_mean(2),pb_mean.norm());
	//ROS_INFO("dir(%f,%f,%f) len: %f",dir(0),dir(1),dir(2),dir.norm());
	//ROS_INFO("orthLineDir(%f,%f,%f) len: %f",orthLineDir(0),orthLineDir(1),orthLineDir(2),orthLineDir.norm());
	//ROS_INFO("d = %f",d);
	//ROS_INFO("point_a(%f,%f,%f)",point_a(0),point_a(1),point_a(2));

	dir_x = dir(0);
	dir_y = dir(1);
	dir_z = dir(2); 
	point_x = point_a(0);
	point_y = point_a(1);
	point_z = point_a(2);
}

Line::Line(){id = line_counter++;}
Line::~Line(){}

float Line::distance(Point * p){					return distance(p->pos);}
float Line::distance(Vector3f & point){				return ((point - point_a).cross(point - point_b)).norm();}
float Line::distance(float x, float y, float z){
	Vector3f p = Vector3f(x,y,z);
	return distance(p);
}

void Line::distance(float & de, float & t1, float & t2, Line * line){
	Eigen::Vector3f P21 = point_a-line->point_a;
	Eigen::Vector3f M = point_b.cross(line->point_b);
	float m2 = M.dot(M);
	Eigen::Vector3f R0 = P21.cross(M);
	Eigen::Vector3f R1 = P21.cross(M/sqrt(m2));
	Eigen::Vector3f R2 = P21.cross(M/m2);

	t1 = R2.dot(point_b);
	t2 = R2.dot(line->point_b);
	de = fabs(P21.dot(M))/sqrt(m2);
/*
	printf("de: %f new: %f\n",de,fabs(P21.dot((point_b.cross(line->point_b)).normalized())));

	printf("P21 = %5.5f %5.5f %5.5f -> %5.5f\n",	P21[0],	P21[1],	P21[2], P21.norm());
	printf("M   = %5.5f %5.5f %5.5f -> %5.5f\n",	M[0],	M[1],	M[2],   M.norm());
	printf("R   = %5.5f %5.5f %5.5f -> %5.5f\n",  	R2[0],  R2[1],  R2[2],  R2.norm());
*/
	//printf("R0 = %f %f %f -> %f %f\n",R0[0],R0[1],R0[2],R0.dot(point_b),R0.dot(line->point_b));
	//printf("R1 = %f %f %f -> %f %f\n",R1[0],R1[1],R1[2],R1.dot(point_b),R1.dot(line->point_b));
	//printf("R2  = %f %f %f -> %f %f\n",R2[0],R2[1],R2[2],R2.dot(point_b),R2.dot(line->point_b));
/*
	printf("1/P21.norm():%3.6f\n",  1.0/P21.norm());
	printf("1/R0.norm():  %3.6f\n",	1.0/R0.norm());
	printf("1/R1.norm():  %3.6f\n",	1.0/R1.norm());
	printf("1/R2.norm():  %3.6f\n",	1.0/R2.norm());
	printf("1/m2:        %3.6f\n",	1.0/m2);
	printf("1/sqrt(m2):  %3.6f\n",	1.0/sqrt(m2));
*/
/*
1/P21.norm():20.448090
1/R.norm():  9.801185
1/dir.norm():0.877685
1/m2:        4.724054
1/sqrt(m2):  2.173489
Original de: 0.013713
t1: -0.115754 t2: -0.100335 relative: 0.866796
s: 0.110254 t: 0.127338 fraction: 0.865836
Div: -1.100080
*/

	//printf("t1: %f t2: %f: de: %f\n",t1,t2,de);
}

void Line::closestPoints(Vector3f & src_point, Vector3f & dst_point,float & s ,float & t, Line * line){
	Vector3f a = point_a;
	Vector3f b = point_b;
	Vector3f c = line->point_a;
	Vector3f d = line->point_b;

	Vector3f ab = (a-b);
	Vector3f cd = (c-d);

	double aba	= ab.dot(a);
	double abc	= ab.dot(c);
	double abcd	= ab.dot(cd);

	double cda	= cd.dot(a);
	double cdc	= cd.dot(c);

	double c1	= aba+abc;
	double c3	= -abcd;

	double c4	= cda+cdc;
	double c5	= abcd;


	s			= -(c4*c3+c1)/(1+c5*c3);
	t			= c4+c5*s;
	src_point	= a+ab*s;
	dst_point	= c+cd*t;

	//s *= dir_length;
	//t *= dir_length;
}

void Line::closestPoint(Vector3f & P, float & t,  Vector3f & Q){

	Vector3f a = point_a;
	Vector3f b = point_b;

	Vector3f ab = (a-b);

	t = ab.dot(P-a);
	Q	= a+ab*t;
	//t *= dir_length;
}
