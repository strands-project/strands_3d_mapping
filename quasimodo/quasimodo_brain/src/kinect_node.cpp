
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

void addAndUpdate(pcl::PointCloud<pcl::PointXYZRGB> & cloud){
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

	reglib::RGBDFrame * frame = new reglib::RGBDFrame(camera,rgb,depth,0, Eigen::Matrix4d::Identity());
	frames.push_back(frame);
	reglib::Model * model		= new reglib::Model(frame,mask);
	models.push_back(model);
	updaters->fuse(model,Eigen::Matrix4d::Identity(),-1);

	cv::namedWindow(	"rgb", cv::WINDOW_AUTOSIZE );
	cv::imshow(			"rgb", rgb );
	cv::waitKey(30);
//	updaters->show();

}

std::string path = "./";

void  cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
	ROS_INFO("pointcloud in %i",counter);
	if(counter % 5 == 4){
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg (*input, cloud);
		//addAndUpdate(cloud);
		char buf[1024];
		sprintf(buf,"%s%.10i.pcd",path.c_str(),counter);
		printf("saving: %s\n",buf);
		pcl::io::savePCDFileBinary (string(buf), cloud);

		cv::Mat rgb;
		rgb.create(cloud.height,cloud.width,CV_8UC3);
		unsigned char * rgbdata = (unsigned char *)rgb.data;
		unsigned int nr_data = cloud.height * cloud.width;
		for(unsigned int i = 0; i < nr_data; i++){
			pcl::PointXYZRGB p = cloud.points[i];
			rgbdata[3*i+0]	= p.b;
			rgbdata[3*i+1]	= p.g;
			rgbdata[3*i+2]	= p.r;
		}
		cv::namedWindow(	"rgb", cv::WINDOW_AUTOSIZE );
		cv::imshow(			"rgb", rgb );
		cv::waitKey(30);
	}
	counter++;
}

int main(int argc, char **argv){

	camera				= new reglib::Camera();
	registration		= new reglib::RegistrationPPR();
	updaters			= new reglib::ModelUpdaterBasicFuse( new reglib::RegistrationPPR() );

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem();
	viewer->setBackgroundColor(0.8,0.8,0.8);
	updaters->viewer	= viewer;

	string path;
	if(argc < 2){	path = "/camera/depth_registered/points";}
	else{			path = string(argv[1]);}

	printf("loading from: %s\n",path.c_str());
	ros::init(argc, argv, "quasimodo_kinect_node");
	ros::NodeHandle n;
	ros::Subscriber sub2 = n.subscribe (path, 0, cloud_cb);
	ros::spin();

	return 0;
}
