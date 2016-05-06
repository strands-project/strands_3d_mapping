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

int objcounter = 0;
int counter = 0;
int savecounter = 0;
std::string path = "./dataset";

std::string object = "background";

void  cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
	counter++;
	if(counter % 5 == 0){
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg (*input, cloud);
	//	char buf[1024];
	//	sprintf(buf,"%s%.10i.pcd",path.c_str(),counter);
	//	printf("saving: %s\n",buf);
	//	pcl::io::savePCDFileBinary (string(buf), cloud);



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
			depthdata[i]	= short(5000.0 * p.z);
		}

		cv::namedWindow(	"rgb", cv::WINDOW_AUTOSIZE );
		cv::imshow(			"rgb", rgb );
		cv::namedWindow(	"depth", cv::WINDOW_AUTOSIZE );
		cv::imshow(			"depth", depth );
		int res = cv::waitKey(30);
		//printf("res: %i\n",res);
		if(res==10){
			char buf [1024];
			sprintf(buf,"%s/%s/RGB%5.5i.png",path.c_str(),object.c_str(),savecounter);
			imwrite(buf , rgb );

			sprintf(buf,"%s/%s/DEPTH%5.5i.png",path.c_str(),object.c_str(),savecounter);
			imwrite(buf , depth );

			printf("saved: %s/%s/RGB%5.5i.png\n",path.c_str(),object.c_str(),savecounter);
			printf("saved: %s/%s/DEPTH%5.5i.png\n",path.c_str(),object.c_str(),savecounter);

			savecounter++;
		}

		if(res==110){
			savecounter = 0;

			char str [80];
			sprintf(str,"object%i",objcounter);
			objcounter++;
			//printf ("Enter object name: ");
			//scanf ("%79s",str);
			printf ("current object: %s\n",str);
			object = string(str);


			char buf[1024];
			sprintf(buf,"mkdir %s/%s",path.c_str(),object.c_str());
			int retval = system(buf);
		}
	}
}

int main(int argc, char **argv){
	if(argc > 1){	path = string(argv[1]);}

	char buf[1024];
	sprintf(buf,"mkdir %s",path.c_str());
	int retval = system(buf);
	sprintf(buf,"mkdir %s/%s",path.c_str(),object.c_str());
	retval = system(buf);

	string camerapath = "/camera/depth_registered/points";
	if(argc > 2){	camerapath = string(argv[2]);}

	printf("loading from camerapath: %s\n",camerapath.c_str());
	ros::init(argc, argv, "quasimodo_build_dataset_node");
	ros::NodeHandle n;
	ros::Subscriber sub2 = n.subscribe (camerapath, 0, cloud_cb);
	ros::spin();

	return 0;
}
