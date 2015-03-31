#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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
	
	//Read data into images and save.

	int width = input_cloud.width;
	int height = input_cloud.height;

	IplImage * rgb_img							= cvCreateImage(cvSize(input_cloud.width, input_cloud.height), IPL_DEPTH_8U, 3);
	char * rgb_data								= (char *)(rgb_img->imageData);
	IplImage * depth_img						= cvCreateImage(cvSize(input_cloud.width, input_cloud.height), IPL_DEPTH_16U, 1);
	unsigned short * depth_data					= (unsigned short *)(depth_img->imageData);

	for(int w = 0; w < width; w++){
		for(int h = 0; h < height; h++){
			int ind = h*input_cloud.width + w;
			rgb_data[3*ind+0] = int(input_cloud.points[ind].b);
			rgb_data[3*ind+1] = int(input_cloud.points[ind].g);
			rgb_data[3*ind+2] = int(input_cloud.points[ind].r);
			depth_data[ind]   = (unsigned short)(5000*input_cloud.points[ind].z);
		}
	}

	char buf[1024];

	sprintf(buf,"%s/RGB%.10i.png",path.c_str(),counter);
	if(!cvSaveImage(buf,rgb_img)){printf("Could not save: %s\n",buf);}
	cvReleaseImage( &rgb_img);
	sprintf(buf,"%s/Depth%.10i.png",path.c_str(),counter);
	if(!cvSaveImage(buf,depth_img)){printf("Could not save: %s\n",buf);}
	cvReleaseImage( &depth_img );

}

int main(int argc, char **argv)
{
	printf("starting image recording software\n");
	if(argc == 1){
		printf("Run program with second argument to define path for the files\n");
		return -1;
	}
	path = string(argv[1]);

	ros::init(argc, argv, "ekz_record");
	ros::NodeHandle n;
	ros::Subscriber sub2 = n.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
	ros::spin();
	return 0;
}
