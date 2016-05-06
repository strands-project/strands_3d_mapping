
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

//#include "modelupdater/ModelUpdater.h"
#include "modelupdater2/ModelUpdater2.h"
#include "core/RGBDFrame.h"

#include <string.h>

using namespace std;

int counter = 0;

reglib::Camera *						camera;
//reglib::Registration *					registration;
std::vector<reglib::RGBDFrame *>		frames;
//std::vector<reglib::Model *>			models;
//reglib::ModelUpdater *					updaters;
reglib::ModelUpdater2 *					updaters2;

std::vector<cv::Mat> rgb_images;
std::vector<cv::Mat> depth_images;
std::vector<double> timestamps;

void addFBdata(string path, int start, int stop, int skip){
	string input 		= path;
	string input_rgb	= string(input+"/rgb.txt");
	string input_depth	= string(input+"/depth.txt");
	//string input_gt		= string(input+"/groundtruth.txt");
	printf("input:       %s\n",input.c_str());
	printf("input_rgb:   %s\n",input_rgb.c_str());
	printf("input_depth: %s\n",input_depth.c_str());


	int camera_id = 0;
	if(input.find("rgbd_dataset_freiburg1") != -1){camera_id = 1;}
	if(input.find("rgbd_dataset_freiburg2") != -1){camera_id = 2;}
	if(input.find("rgbd_dataset_freiburg3") != -1){camera_id = 3;}
	printf("Camer_id: %i\n",camera_id);

	camera->idepth_scale = 1.0/5000.0;

	if(camera_id == 0){
		camera->fx			= 525.0;				//Focal Length X
		camera->fy			= 525.0;				//Focal Length Y
		camera->cx			= 319.5;				//Center coordinate X
		camera->cy			= 239.5;				//Center coordinate X
	}else if(camera_id == 1){
		camera->fx			= 517.3;				//Focal Length X
		camera->fy			= 516.5;				//Focal Length Y
		camera->cx			= 318.6;				//Center coordinate X
		camera->cy			= 255.3;				//Center coordinate X
	}else if(camera_id == 2){
		camera->fx			= 520.9;				//Focal Length X
		camera->fy			= 521.0;				//Focal Length Y
		camera->cx			= 325.1;				//Center coordinate X
		camera->cy			= 249.7;				//Center coordinate X
	}else if(camera_id == 3){
		camera->fx			= 535.4;				//Focal Length X
		camera->fy			= 539.2;				//Focal Length Y
		camera->cx			= 320.1;				//Center coordinate X
		camera->cy			= 247.6;				//Center coordinate X
	}else{printf("Error, should not get to here\n");}

	string line;

	ifstream rgb_file (input_rgb.c_str());
	vector<pair<double, string> > rgb_lines;
	if (rgb_file.is_open()){
		while ( rgb_file.good()){
			getline (rgb_file,line);
			if(line[0] != '#'){
				int space1 = line.find(" ");
				if(space1 != -1){
					int dot1		= line.find(".");
					string secs		= line.substr(0,dot1);
					string nsecs	= line.substr(dot1+1,space1-dot1);
					string path		= line.substr(space1+1);
					double timestamp = double(atoi(secs.c_str()))+0.000001*double(atoi(nsecs.c_str()));
					rgb_lines.push_back(make_pair(timestamp,path));
				}
			}
		}
		rgb_file.close();
	}else{cout << "Unable to open " << input;}

	ifstream depth_file (input_depth.c_str());
	vector<pair<double, string> > depth_lines;
	if (depth_file.is_open()){
		while ( depth_file.good()){
			getline (depth_file,line);
			if(line[0] != '#'){
				int space1 = line.find(" ");
				if(space1 != -1){
					int dot1		= line.find(".");
					string secs		= line.substr(0,dot1);
					string nsecs	= line.substr(dot1+1,space1-dot1);
					string path		= line.substr(space1+1);
					double timestamp = double(atoi(secs.c_str()))+0.000001*double(atoi(nsecs.c_str()));
					depth_lines.push_back(make_pair(timestamp,path));
				}
			}
		}
		depth_file.close();
	}else{cout << "Unable to open " << input;}

	unsigned int rgb_counter = 0;
	unsigned int depth_counter = 0;

	vector<int> rgb_indexes;
	vector<int> depth_indexes;

	float max_diff = 0.015;

	for(; rgb_counter < rgb_lines.size(); rgb_counter++){
		double rgb_ts		= rgb_lines.at(rgb_counter).first;
		double depth_ts		= depth_lines.at(depth_counter).first;
		double diff_best	= fabs(rgb_ts - depth_ts);

		for(unsigned int current_counter = depth_counter; current_counter < depth_lines.size(); current_counter++){
			double dts = depth_lines.at(current_counter).first;
			double diff_current = fabs(rgb_ts - dts);
			if(diff_current <= diff_best){
				diff_best = diff_current;
				depth_counter = current_counter;
				depth_ts = dts;
			}else{break;}
		}
		if(diff_best > max_diff){continue;}//Failed to find corresponding depth image
		rgb_indexes.push_back(rgb_counter);
		depth_indexes.push_back(depth_counter);
	}


	for(unsigned int i = start; i < rgb_indexes.size() && i < stop; i+= skip){
		string rgbpath = input+"/"+rgb_lines.at(rgb_indexes.at(i)).second;
		string depthpath = input+"/"+depth_lines.at(depth_indexes.at(i)).second;

		cv::Mat rgbimage    = cv::imread(rgbpath, CV_LOAD_IMAGE_COLOR);
		cv::Mat depthimage  = cv::imread(depthpath, CV_LOAD_IMAGE_UNCHANGED);

		rgb_images.push_back(rgbimage);
		depth_images.push_back(depthimage);
		timestamps.push_back(depth_lines.at(depth_indexes.at(i)).first);

		std::cout << "\rloaded " << i << " " << std::cout.flush();
	}
}
/*
void saveModelToFB(reglib::Model * model, string path = "testoutput.txt"){
	printf("Saving map in: %s\n",path.c_str());

	ofstream myfile;
	myfile.open(path.c_str());

	char buf[1000];
	for(int i = 0; i < model->frames.size(); i++){
		printf("pose: %i\n",i);

		Matrix4d p = model->relativeposes[i];
		Eigen::Affine3d a(p.cast<double>());
		Eigen::Quaterniond qr(a.rotation());
		double timestamp = model->frames[i]->capturetime;//frames.at(i)->input->rgb_timestamp;
		float tx = a(0,3);
		float ty = a(1,3);
		float tz = a(2,3);
		float qx = qr.x();
		float qy = qr.y();
		float qz = qr.z();
		float qw = qr.w();
		int n = sprintf(buf,"%f %f %f %f %f %f %f %f\n",timestamp,tx,ty,tz,qx,qy,qz,qw);
		myfile << buf;
	}

	myfile.close();

}
*/
void addAndUpdate(cv::Mat rgb, cv::Mat depth, double timestamp){
	unsigned int nr_data = 640*480;
	cv::Mat mask;
	mask.create(480,640,CV_8UC1);
	unsigned char * maskdata = (unsigned char *)mask.data;
	for(unsigned int i = 0; i < nr_data; i++){maskdata[i] = 255;}

	reglib::RGBDFrame	* frame	= new reglib::RGBDFrame(camera,rgb,depth,timestamp, Eigen::Matrix4d::Identity(),false);
	updaters2->addFrame(frame,mask);
	//reglib::Model		* model	= new reglib::Model(frame,mask);
	//frames.push_back(frame);
//	models.push_back(model);
/*
	if(updaters->model->frames.size() == 13){
		//registration->setVisualizationLvl(3);
	}

	Eigen::Matrix4d ig = Eigen::Matrix4d::Identity();
	if(updaters->model->frames.size() > 0){ig = updaters->model->relativeposes.back();}
	if(updaters->model->frames.size() > 1){ig *= updaters->model->relativeposes[updaters->model->frames.size()-2].inverse() * ig;}
	updaters->fuse(model,ig,-1);

	if(updaters->model->frames.size() % 10 == 0){
		//updaters->show(true);
	}else{
		//updaters->show(true);
	}

	cv::namedWindow(	"rgb", cv::WINDOW_AUTOSIZE );
	cv::imshow(			"rgb", rgb );
	cv::waitKey(100);
*/
}

int main(int argc, char **argv){
	camera				= new reglib::Camera();
//	addFBdata("/home/johane/Downloads/rgbd_dataset_freiburg3_nostructure_texture_near_withloop", 1, 3, 1);
	addFBdata(argv[1], 1, 1500, 1);

	updaters2			= new reglib::ModelUpdater2();

	for(int i = 0; i < rgb_images.size(); i++){
		addAndUpdate(rgb_images[i],depth_images[i],timestamps[i]);
	}

/*
	registration		= new reglib::RegistrationPPR();
	updaters			= new reglib::ModelUpdaterBasicFuse( registration );

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem();
	viewer->setBackgroundColor(0.8,0.8,0.8);
	updaters->viewer	= viewer;



	saveModelToFB(updaters->model,"test.txt");
*/
/*
	for(unsigned int i = 1; i < argc; i++){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[i], *cloud);
		addAndUpdate(*cloud);
	}
	updaters->show();
*/
	return 0;
}
