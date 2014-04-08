#include "FrameMatcher.h"

using namespace std;

void FrameMatcher::setVerbose(bool b){verbose = b;}
void FrameMatcher::setDebugg(bool b){debugg = b;}

FrameMatcher::~FrameMatcher(){}
Transformation * FrameMatcher::getTransformation(RGBDFrame * src, RGBDFrame * dst){

	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = Eigen::Matrix4f::Identity();
	transformation->src = src;
	transformation->dst = dst;
	transformation->weight = 0;
	return transformation;
}
void FrameMatcher::print(){printf("%s\n",name.c_str());}
void FrameMatcher::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
