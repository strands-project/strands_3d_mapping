#include "myhull.h"
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>

//#include "ICP.h"
namespace reglib
{
myhull::myhull(){}

void myhull::compute(std::vector< Eigen::VectorXd > & input, double alpha){

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	cloud_hull = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

	if(input.size() > 5){
		cloud->points.resize(input.size());
		for(unsigned int i = 0; i < input.size(); i++){
			cloud->points[i].x = input[i](0);
			cloud->points[i].y = input[i](1);
			cloud->points[i].z = input[i](2);
		}

		//pcl::ConvexHull<pcl::PointXYZ> chull;


		pcl::ConcaveHull<pcl::PointXYZ> chull;
		chull.setAlpha (alpha);

		chull.setInputCloud (cloud);
		chull.reconstruct (*cloud_hull,polygons);
	}
	//exit(0);

}

bool myhull::isInlier( Eigen::VectorXd input){
//	return true;

	if(cloud_hull->points.size() == 0){return false;}

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud (new pcl::PointCloud<pcl::PointXYZ>);
	inputcloud->points.resize(1);
	inputcloud->points[0].x = input(0);
	inputcloud->points[0].y = input(1);
	inputcloud->points[0].z = input(2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::CropHull<pcl::PointXYZ> crop_filter;
	crop_filter.setInputCloud (inputcloud);
	crop_filter.setHullCloud (cloud_hull);
	crop_filter.setHullIndices (polygons);
	crop_filter.setDim (3);
	crop_filter.filter (*output);

	if(output->points.size() == 0){return false;}
	return true;

}

myhull::~myhull(){}
}
