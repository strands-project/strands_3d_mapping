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

//Opencv
#include "cv.h"
#include "highgui.h"
#include <opencv2/opencv.hpp>

#include <string.h>


using namespace std;

string path;
int counter = 0;

void  cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
	counter++;
	ROS_INFO("pointcloud in %i",counter);
    
	pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
	pcl::fromROSMsg (*input, input_cloud);

	char buf[1024];
	sprintf(buf,"%s%.10i.pcd",path.c_str(),counter);
	pcl::io::savePCDFileBinary (string(buf), input_cloud);
	
}


int main(int argc, char **argv)
{
	printf("starting pcd_recording software\n");
	if(argc == 1){
		printf("Run program with second argument to define path and start of filename for the .pcd files\n");
		return -1;
	}
	path = string(argv[1]);

	ros::init(argc, argv, "ekz_record");
	ros::NodeHandle n;
	ros::Subscriber sub2 = n.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
	ros::spin();
	return 0;
}
