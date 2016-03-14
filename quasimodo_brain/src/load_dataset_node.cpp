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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "quasimodo_msgs/model.h"
#include "modelupdater/ModelUpdater.h"
#include <sensor_msgs/PointCloud2.h>
#include <string.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "eigen_conversions/eigen_msg.h"
#include "tf_conversions/tf_eigen.h"

#include "metaroom_xml_parser/simple_xml_parser.h"
#include "metaroom_xml_parser/simple_summary_parser.h"

#include <tf_conversions/tf_eigen.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"

#include "ros/ros.h"
#include <quasimodo_msgs/query_cloud.h>
#include <quasimodo_msgs/visualize_query.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include "quasimodo_msgs/model.h"
#include "quasimodo_msgs/rgbd_frame.h"
#include "quasimodo_msgs/model_from_frame.h"
#include "quasimodo_msgs/index_frame.h"
#include "quasimodo_msgs/fuse_models.h"
#include "quasimodo_msgs/get_model.h"


using namespace std;

int counter = 0;
int savecounter = 0;
std::string path = "./dataset";

std::string object = "background";
/*
void  cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input){
	counter++;
	if(counter % 5 == 0){
		pcl::PointCloud<pcl::PointXYZRGB> cloud;
		pcl::fromROSMsg (*input, cloud);

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
			sprintf(buf,"%s/%s/RGB%5.5i.png\n",path.c_str(),object.c_str(),savecounter);
			imwrite(buf , rgb );

			sprintf(buf,"%s/%s/DEPTH%5.5i.png\n",path.c_str(),object.c_str(),savecounter);
			imwrite(buf , depth );

			printf("saved: %s/%s/RGB%5.5i.png\n",path.c_str(),object.c_str(),savecounter);
			printf("saved: %s/%s/DEPTH%5.5i.png\n",path.c_str(),object.c_str(),savecounter);

			savecounter++;
		}

		if(res==110){
			savecounter = 0;

			char str [80];

			printf ("Enter object name: ");
			scanf ("%79s",str);
			printf ("current object: %s\n",str);
			object = string(str);


			char buf[1024];
			sprintf(buf,"mkdir %s/%s",path.c_str(),object.c_str());
			int retval = system(buf);
		}
	}
}
*/

vector<string> do_path(string path){
	vector<string> ret;
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (path.c_str())) != NULL) {
		while ((ent = readdir (dir)) != NULL) {
			string filename = ent->d_name;
			if(filename.front() != '.'){
				if(ent->d_type == DT_DIR && (filename.compare("background") != 0)){
					printf("adding: %s/%s\n",path.c_str(),filename.c_str());
					ret.push_back(filename);
				}
			}
		}
		closedir (dir);
	}
	return ret;
}

using namespace cv;
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>

/// Global variables
Mat src, erosion_dst, dilation_dst;

int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

/** Function Headers */
void Erosion( int, void* );
void Dilation( int, void* );

/**  @function Erosion  */
void Erosion( int, void* )
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
									   Size( 2*erosion_size + 1, 2*erosion_size+1 ),
									   Point( erosion_size, erosion_size ) );

  /// Apply the erosion operation
  erode( src, erosion_dst, element );
  imshow( "Erosion Demo", erosion_dst );
}

/** @function Dilation */
void Dilation( int, void* )
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
									   Size( 2*dilation_size + 1, 2*dilation_size+1 ),
									   Point( dilation_size, dilation_size ) );
  /// Apply the dilation operation
  dilate( src, dilation_dst, element );
  imshow( "Dilation Demo", dilation_dst );
}

vector<cv::Mat> masks;
vector<cv::Mat> rgbs;
vector<cv::Mat> depths;
vector<int> inds;

ros::ServiceClient model_from_frame_client;
ros::ServiceClient fuse_models_client;
ros::ServiceClient get_model_client;
ros::ServiceClient index_frame_client;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());

  for (unsigned int i = 0; i < rgbs.size(); i++){
      printf("start adding frame %i\n",i);

      cv::Mat maskimage	= masks[i];
      cv::Mat rgbimage	= rgbs[i];
      cv::Mat depthimage	= depths[i];

      geometry_msgs::Pose		pose;
      tf::poseEigenToMsg (Eigen::Affine3d::Identity(), pose);

      cv_bridge::CvImage rgbBridgeImage;
      rgbBridgeImage.image = rgbimage;
      rgbBridgeImage.encoding = "bgr8";

      cv_bridge::CvImage depthBridgeImage;
      depthBridgeImage.image = depthimage;
      depthBridgeImage.encoding = "mono16";

      quasimodo_msgs::index_frame ifsrv;
      ifsrv.request.frame.capture_time = ros::Time();
      ifsrv.request.frame.pose		= pose;
      ifsrv.request.frame.frame_id	= -1;
      ifsrv.request.frame.rgb			= *(rgbBridgeImage.toImageMsg());
      ifsrv.request.frame.depth		= *(depthBridgeImage.toImageMsg());

      if (index_frame_client.call(ifsrv)){//Add frame to model server
          int frame_id = ifsrv.response.frame_id;
          ROS_INFO("frame_id%i", frame_id );
       }else{ROS_ERROR("Failed to call service index_frame");}

      printf("stop adding frame %i\n",i);
  }

  for (unsigned int i = 0; i < rgbs.size(); i++){
      printf("start adding mask %i\n",i);
      cv::Mat mask	= masks[i];

      cv_bridge::CvImage maskBridgeImage;
      maskBridgeImage.image = mask;
      maskBridgeImage.encoding = "mono8";

      quasimodo_msgs::model_from_frame mff;
      mff.request.mask = *(maskBridgeImage.toImageMsg());
      mff.request.frame_id = i;

      if (model_from_frame_client.call(mff)){//Build model from frame
          int model_id = mff.response.model_id;
          if(model_id > 0){
              ROS_INFO("model_id%i", model_id );
          }
      }else{ROS_ERROR("Failed to call service index_frame");}
      printf("stop adding mask %i\n",i);
  }

}

int main(int argc, char **argv){

	if(argc > 1){	path = string(argv[1]);}

	char buf[1024];
	sprintf(buf,"%s/background",path.c_str());
	printf("%s\n",buf);

	cv::Mat bg;
	for(int i = 0; true ; i++){

		sprintf(buf,"%s/%s/RGB%5.5i.png",path.c_str(),object.c_str(),i);
		printf("%s\n",buf);
		cv::Mat rgb = cv::imread(buf, -1);   // Read the file

		sprintf(buf,"%s/%s/DEPTH%5.5i.png",path.c_str(),object.c_str(),i);
		printf("%s\n",buf);
		cv::Mat depth = cv::imread(buf, -1);   // Read the file

		if(! rgb.data ){ printf("no rgb, breaking\n");break;}
		if(! depth.data ){printf("nor depth, breaking\n");break;}
		if(i == 0){bg = depth;}
		else{
			unsigned short * bgdata = (unsigned short *)bg.data;
            unsigned short * depthdata = (unsigned short *)depth.data;

			unsigned int nr_data = 640*480;
			for(unsigned int i = 0; i < nr_data; i++){
				if(bgdata[i] == 0){ bgdata[i] = depthdata[i];}
				else if(depthdata[i] != 0){
					bgdata[i] = std::min(bgdata[i],depthdata[i]);
				}
			}
		}

		//cv::namedWindow(	"rgb", cv::WINDOW_AUTOSIZE );
		//cv::imshow(			"rgb", rgb );
		//cv::namedWindow(	"depth", cv::WINDOW_AUTOSIZE );
		//cv::imshow(			"depth", depth );
		//int res = cv::waitKey(0);
	}

	//cv::namedWindow(	"bg", cv::WINDOW_AUTOSIZE );
	//cv::imshow(			"bg", bg );
	unsigned short * bgdata = (unsigned short *)bg.data;
	//int res = cv::waitKey(0);

	vector<string> ret = do_path(path);



	for(int r = 0; r < ret.size() ; r++){
		object = ret[r];
		printf("object: %s\n",object.c_str());
		for(int i = 1; true ; i++){

			sprintf(buf,"%s/%s/RGB%5.5i.png",path.c_str(),object.c_str(),i);
			printf("%s\n",buf);
			cv::Mat rgb = cv::imread(buf, -1);   // Read the file

			sprintf(buf,"%s/%s/DEPTH%5.5i.png",path.c_str(),object.c_str(),i);
			printf("%s\n",buf);
			cv::Mat depth = cv::imread(buf, -1);   // Read the file
			unsigned short * depthdata = (unsigned short *)depth.data;

			if(! rgb.data ){break;}

			cv::Mat mask;
			mask.create(480,640,CV_8UC1);
			unsigned char * maskdata = (unsigned char *)mask.data;
			for(unsigned int i = 0; i < 640*480; i++){maskdata[i] = 0;}

			src = mask;


			for(unsigned int j = 0; j < 640*480; j++){
				unsigned short bgs = bgdata[j];
				unsigned short ds = depthdata[j];
				if(bgs == 0 || ds == 0){continue;}

				double bgz	= double(bgdata[j])/5000.0;
				double z	= double(depthdata[j])/5000.0;
				double bgzi = bgz*bgz;
				double zi	= z*z;
				double dz = (z - bgz)/(bgzi + zi);
				//if(j % 100 == 0){printf("%i -> %i %i -> %f\n",j,bgs, ds,dz);}
				if(dz < -0.015 && z < 1){
					maskdata[j] = 255;
				}else{
					maskdata[j] = 0;
				}

			}


			int filter_size = 2;

			cv::Mat erosion_mask;
			cv::Mat dilation_mask;
			cv::Mat element = getStructuringElement( MORPH_RECT, Size( 2*filter_size + 1, 2*filter_size+1 ), Point( filter_size, filter_size ) );
			erode( mask, erosion_mask, element );

			filter_size = 8;
			element = getStructuringElement( MORPH_RECT, Size( 2*filter_size + 1, 2*filter_size+1 ), Point( filter_size, filter_size ) );
			dilate( erosion_mask, dilation_mask, element );

			unsigned char * dilation_maskdata = (unsigned char *)dilation_mask.data;
			for(unsigned int j = 0; j < 640*480; j++){
				maskdata[j] = maskdata[j]*(dilation_maskdata[j] != 0);
			}

			masks.push_back(mask);
			rgbs.push_back(rgb);
			depths.push_back(depth);
			inds.push_back(r);

			cv::namedWindow(	"mask", cv::WINDOW_AUTOSIZE );
			cv::imshow(			"mask", mask);
			cv::namedWindow(	"rgb", cv::WINDOW_AUTOSIZE );
			cv::imshow(			"rgb", rgb);
			cv::namedWindow(	"depth", cv::WINDOW_AUTOSIZE );
			cv::imshow(			"depth", depth );
            int res = cv::waitKey(30);

		}
	}

	for (unsigned int i = 0; i < rgbs.size(); i++){
		int r = rand()%rgbs.size();

		cv::Mat mask = masks[i];
		cv::Mat rgb = rgbs[i];
		cv::Mat depth = depths[i];
		int ind = inds[i];


		masks[i]	= masks[r];
		rgbs[i]		= rgbs[r];
		depths[i]	= depths[r];
		inds[i]		= inds[r];

		masks[r]	= mask;
		rgbs[r]		= rgb;
		depths[r]	= depth;
		inds[r]		= ind;
	}

	ros::init(argc, argv, "use_rares_client");
	ros::NodeHandle n;

    model_from_frame_client	= n.serviceClient<quasimodo_msgs::model_from_frame>("model_from_frame");
    fuse_models_client		= n.serviceClient<quasimodo_msgs::fuse_models>(		"fuse_models");
    get_model_client			= n.serviceClient<quasimodo_msgs::get_model>(		"get_model");
    index_frame_client		= n.serviceClient<quasimodo_msgs::index_frame>(		"index_frame");

    ros::Subscriber sub = n.subscribe("modelserver", 1000, chatterCallback);
    ros::spin();
    /*
	while(true){
		char str [80];
		printf ("build model?");
		scanf ("%79s",str);
		int nr_todo = atoi(str);
		if(str[0] == 'q'){exit(0);}
	}
    */
	return 0;
}
