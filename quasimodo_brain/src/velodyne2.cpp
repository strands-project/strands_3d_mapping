
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

int counterr = 0;
int counterl = 0;
std::string path = "./";

void  cloud_cb_l(const sensor_msgs::PointCloud2ConstPtr& input){
	ROS_INFO("l pointcloud in %i",counterl);
}

void  cloud_cb_r(const sensor_msgs::PointCloud2ConstPtr& input){
	ROS_INFO("r pointcloud in %i",counterr);
//	if(counter % 5 == 4){
//		pcl::PointCloud<pcl::PointXYZRGB> cloud;
//		pcl::fromROSMsg (*input, cloud);
//		//addAndUpdate(cloud);
//		char buf[1024];
//		sprintf(buf,"%s%.10i.pcd",path.c_str(),counter);
//		printf("saving: %s\n",buf);
//		pcl::io::savePCDFileBinary (string(buf), cloud);

//		cv::Mat rgb;
//		rgb.create(cloud.height,cloud.width,CV_8UC3);
//		unsigned char * rgbdata = (unsigned char *)rgb.data;
//		unsigned int nr_data = cloud.height * cloud.width;
//		for(unsigned int i = 0; i < nr_data; i++){
//			pcl::PointXYZRGB p = cloud.points[i];
//			rgbdata[3*i+0]	= p.b;
//			rgbdata[3*i+1]	= p.g;
//			rgbdata[3*i+2]	= p.r;
//		}
//		cv::namedWindow(	"rgb", cv::WINDOW_AUTOSIZE );
//		cv::imshow(			"rgb", rgb );
//		cv::waitKey(30);
//	}
//	counter++;
}

int main(int argc, char **argv){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem();
	viewer->setBackgroundColor(0.8,0.8,0.8);

	string pathr;
	if(argc < 2){	pathr = "/camera/depth_registered/points";}
	else{			pathr = string(argv[1]);}

	string pathl;
	if(argc < 3){	pathl = "/camera/depth_registered/points";}
	else{			pathl = string(argv[2]);}

	ros::init(argc, argv, "massreg_velodyne_node");
	ros::NodeHandle n;
	ros::Subscriber subr = n.subscribe (path, 0, cloud_cb_r);
	ros::Subscriber subl = n.subscribe (path, 0, cloud_cb_l);
	ros::spin();

	return 0;
}
