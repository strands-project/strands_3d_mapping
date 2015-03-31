#ifndef RGBDSegmentation_H_
#define RGBDSegmentation_H_

//#include "../Frame_input.h"
//#include "cv.h"
//#include "highgui.h"
#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <sys/time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include <string>
#include <iostream>
#include <stdio.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
//#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/pcl_plotter.h>
#include <pcl/console/parse.h>

#include "mygeometry/mygeometry.h"
#include "FrameInput.h"



using namespace Eigen;
using namespace std;

struct segmentation_fit {
	//MyLinkedList<segment_pixel * > * segment;
	Matrix3f covMat;
	Matrix3f U;
	Vector3f S;
	Matrix3f V;
	Vector3f mean;
	int nr_valid;
	vector<int> * seg_w;
	vector<int> * seg_h;
};

class Segments {
	public:
		int ** segment_id;
		vector<Plane * > * planes;
};

class RGBDSegmentation
{
	public:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		//Calibration * calibration;
		virtual ~RGBDSegmentation();
		virtual Segments * segment(FrameInput * fi);
		void disp_xy_edge(string s , float** xedge,float** yedge, int width,int height, bool stop);
		void disp_edge(string s , float** edge, int width,int height, bool stop);
		void disp_segments(int ** segment_id, int width, int height, bool stop);

		float ** getII(float ** input , int width, int height);
		float ** getIIValid(float ** input , int width, int height);

		float getSum(float ** iimg, int topw,int toph, int botw, int both);
		float getAvg(float ** iimg, int topw,int toph, int botw, int both, int width, int height);

		float getAvg(float ** iimg,float ** iiValid, int topw,int toph, int botw, int both, int width, int height);

		void getEdges(float ** & wedges, float ** & hedges, float *** ii, float *** iiValid, int dim , int width, int height,int s1 ,int s2);
		void getEdges(float ** & wedges, float ** & hedges, float *** ii, int dim , int width, int height, int s1 ,int s2);

		void getEdges(float ** & wedges, float ** & hedges, float *** rgb,float *** xyz,float *** nxnynz, int width, int height);

		float** getWEdges(float *** rgb,float *** xyz,float *** nxnynz, int width, int height);
		float** getHEdges(float *** rgb,float *** xyz,float *** nxnynz, int width, int height);
		float** mergeEdges(float ** wedges, float ** hedges, int width, int height);
		void normalize(float ** wedges, float ** hedges, int width, int height);
		float** mul(float ** edges1, float ** edges2, int width, int height);
		float** add(float ** edges1, float ** edges2, int width, int height);
		void trim(float ** edges1, int width, int height);
		void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
};

#endif
