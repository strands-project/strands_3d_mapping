#include "TransformationFilter.h"

using namespace std;

TransformationFilter::~TransformationFilter(){}
Transformation * TransformationFilter::filterTransformation(Transformation * input){
	return input;
}
void TransformationFilter::print(){printf("%s\n",name.c_str());}
void TransformationFilter::setVisualization(boost::shared_ptr<pcl::visualization::PCLVisualizer> view){viewer = view;}
