
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "modelupdater/ModelUpdater.h"
#include "core/RGBDFrame.h"

#include <string.h>

using namespace std;

int counter = 0;

reglib::Camera *						camera;
reglib::Registration *					registration;
std::vector<reglib::RGBDFrame *>		frames;
std::vector<reglib::Model *>			models;
reglib::ModelUpdater *					updaters;

void addAndUpdate(pcl::PointCloud<pcl::PointXYZRGB> cloud){
	cv::Mat rgb;
	rgb.create(cloud.height,cloud.width,CV_8UC3);
	unsigned char * rgbdata = (unsigned char *)rgb.data;

	cv::Mat depth;
	depth.create(cloud.height,cloud.width,CV_16UC1);
	unsigned short * depthdata = (unsigned short *)depth.data;

	unsigned int nr_data = cloud.height * cloud.width;
	for(unsigned int i = 0; i < nr_data; i++){
		pcl::PointXYZRGB p = cloud.points[i];
		rgbdata[3*i+0]	= p.b;
		rgbdata[3*i+1]	= p.g;
		rgbdata[3*i+2]	= p.r;
		depthdata[i]	= p.z*1000.0;
	}

	cv::Mat mask;
	mask.create(cloud.height,cloud.width,CV_8UC1);
	unsigned char * maskdata = (unsigned char *)mask.data;
	for(unsigned int i = 0; i < nr_data; i++){maskdata[i] = 255;}

	reglib::RGBDFrame	* frame	= new reglib::RGBDFrame(camera,rgb,depth,0, Eigen::Matrix4d::Identity());
	reglib::Model		* model	= new reglib::Model(frame,mask);
	frames.push_back(frame);
	models.push_back(model);

	Eigen::Matrix4d ig = Eigen::Matrix4d::Identity();
	if(updaters->model->frames.size() > 0){ig = updaters->model->relativeposes.back();}
	if(updaters->model->frames.size() > 1){ig *= updaters->model->relativeposes[updaters->model->frames.size()-2].inverse() * ig;}
	updaters->fuse(model,ig,-1);

	if(updaters->model->frames.size() % 15 == 0){
		updaters->show(true);
	}else{
		updaters->show(true);
	}

	//updaters->show(false);


	cv::namedWindow(	"rgb", cv::WINDOW_AUTOSIZE );
	cv::imshow(			"rgb", rgb );
	cv::waitKey(100);

}

int main(int argc, char **argv){

	camera				= new reglib::Camera();
	registration		= new reglib::RegistrationPPR();
	updaters			= new reglib::ModelUpdaterBasicFuse( new reglib::RegistrationPPR() );

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem();
	viewer->setBackgroundColor(0.8,0.8,0.8);
	updaters->viewer	= viewer;


	for(unsigned int i = 1; i < argc; i++){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[i], *cloud);
		addAndUpdate(*cloud);
	}
	updaters->show();
	return 0;
}
