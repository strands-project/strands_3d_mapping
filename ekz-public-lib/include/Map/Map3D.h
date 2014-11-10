#ifndef Map3D_H_
#define Map3D_H_

//Standard
#include <queue>
#include <iomanip>
#include <stdio.h>
#include <string.h>
#include <string>
#include <iostream>

//Boost
#include <boost/thread/thread.hpp>

//Mine
#include "FrameInput.h"
#include "RGBDFrame.h"
#include "Transformation.h" 
#include "FrameMatcher.h"
#include "FeatureExtractor.h"
//#include "ekz_g2o.h"


//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
using namespace Eigen;

struct TodoTransformation {
	double priority;
	RGBDFrame * src;
	RGBDFrame * dst;
};

class CompareTodoTransformation {
	public:
    bool operator()(TodoTransformation& t1, TodoTransformation& t2){return t1.priority > t2.priority;}
};

class Map3D
{
	public:
	FrameMatcher * matcher;
	FrameMatcher * loopclosure_matcher;
	FeatureExtractor * extractor;
	RGBDSegmentation * segmentation;
	Calibration * calibration;

	bool verbose;
	
	vector<RGBDFrame *> frames;
	vector<int> largest_component;

	
	vector<Transformation *> transformations;
	vector<Matrix4f> poses;
	
	Map3D();
	virtual ~Map3D(); 

	virtual void addFrame(string rgb_path, string depth_path);
	virtual void addFrame(Calibration * cal,string rgb_path, string depth_path);
	
	virtual void addFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	virtual void addFrame(Calibration * cal, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	
	virtual void addFrame(FrameInput * fi);
	virtual void addFrame(RGBDFrame * frame);
	
	virtual void addTransformation(Transformation * transformation);

	virtual vector<Matrix4f> estimate();

	virtual void setVerbose(bool v);
	virtual void setCalibration(Calibration * cal);
	virtual void setMatcher(FrameMatcher * fm);
	virtual void setLoopClosureMatcher(FrameMatcher * fm);
	virtual void setSegmentation(RGBDSegmentation * seg);
	virtual void setFeatureExtractor(FeatureExtractor * fe);
	
	virtual void loadCalibrationWords(string path,string type, int nr_words);
	
	virtual void savePCD(string path);
	virtual void savePCD(string path,bool randomcolor, bool trajectory, float resolution);
	virtual vector<int> getLargestComponent();
	
/*
	
	bool show;
	vector<double > gt_timestamps;
	vector<GroundTruth > gt_data;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	virtual g2o::SE3Quat getQuat(Eigen::Matrix4f mat);
	virtual unsigned long getCurrentTime();
	virtual vector<int> getLargestComponent();
	virtual void cleanTransformations(float threshold);
	virtual void setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view);
	virtual void visualize();
	virtual void showTuning();
	
	virtual void savePoses(string path);
	virtual void saveFBformat(string path);
	virtual void sortFrames();
	virtual void treePoseEst();
	virtual void drawTraj();
*/
};

#include "Map3Dbow.h"

#endif
