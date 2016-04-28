#include "core/RGBDFrame.h"

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <vtkPolyLine.h>

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <vector>
#include <string>

#include <cv.h>
#include <highgui.h>
#include <fstream>

#include <ctime>


namespace reglib
{
unsigned long RGBDFrame_id_counter;
RGBDFrame::RGBDFrame(){
	id = RGBDFrame_id_counter++;
	capturetime = 0;
	pose = Eigen::Matrix4d::Identity();
}

bool updated = true;
void on_trackbar( int, void* ){updated = true;}

RGBDFrame::RGBDFrame(Camera * camera_, cv::Mat rgb_, cv::Mat depth_, double capturetime_, Eigen::Matrix4d pose_, bool compute_normals){

	//printf("%s LINE:%i\n",__FILE__,__LINE__);

    sweepid = -1;
	id = RGBDFrame_id_counter++;
	camera = camera_;
	rgb = rgb_;
	depth = depth_;
	capturetime = capturetime_;
	pose = pose_;

	IplImage iplimg = rgb_;
	IplImage* img = &iplimg;

	//printf("%s LINE:%i\n",__FILE__,__LINE__);

    int width = img->width;
    int height = img->height;
    int sz = height*width;
	const double idepth			= camera->idepth_scale;
	const double cx				= camera->cx;
	const double cy				= camera->cy;
	const double ifx			= 1.0/camera->fx;
	const double ify			= 1.0/camera->fy;

	connections.resize(1);
	connections[0].resize(1);
	connections[0][0] = 0;

	intersections.resize(1);
	intersections[0].resize(1);
	intersections[0][0] = 0;

	nr_labels = 1;
	labels = new int[width*height];
    for(int i = 0; i < width*height; i++){labels[i] = 0;}


	//printf("%s LINE:%i\n",__FILE__,__LINE__);
	unsigned short * depthdata = (unsigned short *)depth.data;
	unsigned char * rgbdata = (unsigned char *)rgb.data;

	depthedges.create(height,width,CV_8UC1);
	unsigned char * depthedgesdata = (unsigned char *)depthedges.data;

	double t = 0.01;
    for(int w = 0; w < width; w++){
        for(int h = 0; h < height;h++){
			int ind = h*width+w;
			depthedgesdata[ind] = 0;
			double z = idepth*double(depthdata[ind]);
			if(w > 0){
				double z2 = idepth*double(depthdata[ind-1]);
				double info = 1.0/(z*z+z2*z2);
				double diff = fabs(z2-z)*info;
				if(diff > t){depthedgesdata[ind] = 255;}
			}

			if(w < width-1){
				double z2 = idepth*double(depthdata[ind+1]);
				double info = 1.0/(z*z+z2*z2);
				double diff = fabs(z2-z)*info;
				if(diff > t){depthedgesdata[ind] = 255;}
			}

			if(h > 0){
				double z2 = idepth*double(depthdata[ind-width]);
				double info = 1.0/(z*z+z2*z2);
				double diff = fabs(z2-z)*info;
				if(diff > t){depthedgesdata[ind] = 255;}
			}

			if(h < height-1){
				double z2 = idepth*double(depthdata[ind+width]);
				double info = 1.0/(z*z+z2*z2);
				double diff = fabs(z2-z)*info;
				if(diff > t){depthedgesdata[ind] = 255;}
			}

			if(h > 0 && w > 0){
				double z2 = idepth*double(depthdata[ind-width-1]);
				double info = 1.0/(z*z+z2*z2);
				double diff = fabs(z2-z)*info;
				if(diff > t){depthedgesdata[ind] = 255;}
			}

			if(w > 0 && h < height-1){
				double z2 = idepth*double(depthdata[ind+width-1]);
				double info = 1.0/(z*z+z2*z2);
				double diff = fabs(z2-z)*info;
				if(diff > t){depthedgesdata[ind] = 255;}
			}

			if(h > 0 && w < width-1){
				double z2 = idepth*double(depthdata[ind-width+1]);
				double info = 1.0/(z*z+z2*z2);
				double diff = fabs(z2-z)*info;
				if(diff > t){depthedgesdata[ind] = 255;}
			}

			if(h < height-1 && w < width-1){
				double z2 = idepth*double(depthdata[ind+width+1]);
				double info = 1.0/(z*z+z2*z2);
				double diff = fabs(z2-z)*info;
				if(diff > t){depthedgesdata[ind] = 255;}
			}
		}
	}

	//printf("%s LINE:%i\n",__FILE__,__LINE__);
	if(compute_normals){
		normals.create(height,width,CV_32FC3);
		float * normalsdata = (float *)normals.data;

		pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud	(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr	normals (new pcl::PointCloud<pcl::Normal>);
		cloud->width	= width;
		cloud->height	= height;
		cloud->points.resize(width*height);

        for(int w = 0; w < width; w++){
            for(int h = 0; h < height;h++){
				int ind = h*width+w;
				pcl::PointXYZ & p = cloud->points[ind];
				double z = idepth*double(depthdata[ind]);
				if(z > 0){
					p.x = (double(w) - cx) * z * ifx;
					p.y = (double(h) - cy) * z * ify;
					p.z = z;
				}else{
					p.x = NAN;
					p.y = NAN;
					p.z = NAN;
				}
			}
		}

		//printf("%s LINE:%i\n",__FILE__,__LINE__);
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud);

		bool tune = false;
		unsigned char * combidata;
		cv::Mat combined;

		int NormalEstimationMethod = 0;
		int MaxDepthChangeFactor = 20;
		int NormalSmoothingSize = 7;
		int depth_dependent_smoothing = 1;

		ne.setMaxDepthChangeFactor(0.001*double(MaxDepthChangeFactor));
		ne.setNormalSmoothingSize(NormalSmoothingSize);
		ne.setDepthDependentSmoothing (depth_dependent_smoothing);
		ne.compute(*normals);

        for(int w = 0; w < width; w++){
            for(int h = 0; h < height;h++){
				int ind = h*width+w;
				pcl::Normal		p2		= normals->points[ind];
				if(!isnan(p2.normal_x)){
					normalsdata[3*ind+0]	= p2.normal_x;
					normalsdata[3*ind+1]	= p2.normal_y;
					normalsdata[3*ind+2]	= p2.normal_z;
				}else{
					normalsdata[3*ind+0]	= 2;
					normalsdata[3*ind+1]	= 2;
					normalsdata[3*ind+2]	= 2;
				}
			}
		}

		//printf("%s LINE:%i\n",__FILE__,__LINE__);
		if(tune){
			combined.create(height,2*width,CV_8UC3);
			combidata = (unsigned char *)combined.data;

			cv::namedWindow( "normals", cv::WINDOW_AUTOSIZE );
			cv::namedWindow( "combined", cv::WINDOW_AUTOSIZE );

			//cv::createTrackbar( "NormalEstimationMethod", "combined", &NormalEstimationMethod, 3, on_trackbar );
			//cv::createTrackbar( "MaxDepthChangeFactor", "combined", &MaxDepthChangeFactor, 1000, on_trackbar );
			//cv::createTrackbar( "NormalSmoothingSize", "combined", &NormalSmoothingSize, 100, on_trackbar );
			//cv::createTrackbar( "depth_dependent_smoothing", "combined", &depth_dependent_smoothing, 1, on_trackbar );

			//while(true){

				if(NormalEstimationMethod == 0){ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);}
				if(NormalEstimationMethod == 1){ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);}
				if(NormalEstimationMethod == 2){ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);}
				if(NormalEstimationMethod == 3){ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);}

				ne.setMaxDepthChangeFactor(0.001*double(MaxDepthChangeFactor));
				ne.setNormalSmoothingSize(NormalSmoothingSize);
				ne.setDepthDependentSmoothing (depth_dependent_smoothing);
				ne.compute(*normals);
                for(int w = 0; w < width; w++){
                    for(int h = 0; h < height;h++){
						int ind = h*width+w;
						pcl::Normal		p2		= normals->points[ind];
						if(!isnan(p2.normal_x)){
							normalsdata[3*ind+0]	= p2.normal_x;
							normalsdata[3*ind+1]	= p2.normal_y;
							normalsdata[3*ind+2]	= p2.normal_z;
						}else{
							normalsdata[3*ind+0]	= 2;
							normalsdata[3*ind+1]	= 2;
							normalsdata[3*ind+2]	= 2;
						}
					}
				}


                for(int w = 0; w < width; w++){
                    for(int h = 0; h < height;h++){
						int ind = h*width+w;
						int indn = h*2*width+(w+width);
						int indc = h*2*width+(w);
						combidata[3*indc+0]	= rgbdata[3*ind+0];
						combidata[3*indc+1]	= rgbdata[3*ind+1];
						combidata[3*indc+2]	= rgbdata[3*ind+2];
						pcl::Normal		p2		= normals->points[ind];
						if(!isnan(p2.normal_x)){
							combidata[3*indn+0]	= 255.0*fabs(p2.normal_x);
							combidata[3*indn+1]	= 255.0*fabs(p2.normal_y);
							combidata[3*indn+2]	= 255.0*fabs(p2.normal_z);
						}else{
							combidata[3*indn+0]	= 255;
							combidata[3*indn+1]	= 255;
							combidata[3*indn+2]	= 255;
						}
					}
				}
				char buf [1024];
                sprintf(buf,"combined%i.png",int(id));
				cv::imwrite( buf, combined );
				printf("saving: %s\n",buf);
				cv::imshow( "combined", combined );
				cv::waitKey(0);
				//while(!updated){cv::waitKey(50);}
				updated = false;
			//}
		}

		//printf("%s LINE:%i\n",__FILE__,__LINE__);
	}
	//show(true);

	//printf("%s LINE:%i\n",__FILE__,__LINE__);
}

RGBDFrame::~RGBDFrame(){}

void RGBDFrame::show(bool stop){
	cv::namedWindow( "rgb", cv::WINDOW_AUTOSIZE );			cv::imshow( "rgb", rgb );
	cv::namedWindow( "normals", cv::WINDOW_AUTOSIZE );		cv::imshow( "normals", normals );
	cv::namedWindow( "depth", cv::WINDOW_AUTOSIZE );		cv::imshow( "depth", depth );
	cv::namedWindow( "depthedges", cv::WINDOW_AUTOSIZE );	cv::imshow( "depthedges", depthedges );
	if(stop){	cv::waitKey(0);}else{					cv::waitKey(30);}
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr RGBDFrame::getPCLcloud(){
	unsigned char * rgbdata = (unsigned char *)rgb.data;
	unsigned short * depthdata = (unsigned short *)depth.data;

	const unsigned int width	= camera->width; const unsigned int height	= camera->height;
	const double idepth			= camera->idepth_scale;
	const double cx				= camera->cx;		const double cy				= camera->cy;
	const double ifx			= 1.0/camera->fx;	const double ify			= 1.0/camera->fy;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr	cloud	(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr		normals (new pcl::PointCloud<pcl::Normal>);
	cloud->width	= width;
	cloud->height	= height;
	cloud->points.resize(width*height);

	for(unsigned int w = 0; w < width; w++){
		for(unsigned int h = 0; h < height;h++){
			int ind = h*width+w;
			double z = idepth*double(depthdata[ind]);
			if(z > 0){
				pcl::PointXYZRGB p;
				p.x = (double(w) - cx) * z * ifx;
				p.y = (double(h) - cy) * z * ify;
				p.z = z;
				p.b = rgbdata[3*ind+0];
				p.g = rgbdata[3*ind+1];
				p.r = rgbdata[3*ind+2];
				cloud->points[ind] = p;
			}
		}
	}

	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	cloud_ptr->width	= width;
	cloud_ptr->height	= height;
	cloud_ptr->points.resize(width*height);

	for(unsigned int w = 0; w < width; w++){
		for(unsigned int h = 0; h < height;h++){
			int ind = h*width+w;
			pcl::PointXYZRGBNormal & p0	= cloud_ptr->points[ind];
			pcl::PointXYZRGB p1			= cloud->points[ind];
			pcl::Normal p2				= normals->points[ind];
			p0.x		= p1.x;
			p0.y		= p1.y;
			p0.z		= p1.z;
			p0.r		= p1.r;
			p0.g		= p1.g;
			p0.b		= p1.b;
			p0.normal_x	= p2.normal_x;
			p0.normal_y	= p2.normal_y;
			p0.normal_z	= p2.normal_z;
		}
	}
	return cloud_ptr;
}

void RGBDFrame::savePCD(std::string path){
    unsigned char * rgbdata = (unsigned char *)rgb.data;
    unsigned short * depthdata = (unsigned short *)depth.data;

    const unsigned int width	= camera->width; const unsigned int height	= camera->height;
    const double idepth			= camera->idepth_scale;
    const double cx				= camera->cx;		const double cy				= camera->cy;
    const double ifx			= 1.0/camera->fx;	const double ify			= 1.0/camera->fy;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr	cloud	(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width	= width;
    cloud->height	= height;
    cloud->points.resize(width*height);

    for(unsigned int w = 0; w < width; w++){
        for(unsigned int h = 0; h < height;h++){
            int ind = h*width+w;
            double z = idepth*double(depthdata[ind]);
            if(z > 0){
                pcl::PointXYZRGB p;
                p.x = (double(w) - cx) * z * ifx;
                p.y = (double(h) - cy) * z * ify;
                p.z = z;
                p.b = rgbdata[3*ind+0];
                p.g = rgbdata[3*ind+1];
                p.r = rgbdata[3*ind+2];
                cloud->points[ind] = p;
            }
        }
    }
    int success = pcl::io::savePCDFileBinaryCompressed(path,*cloud);
}

void RGBDFrame::save(std::string path){
	//printf("saving frame %i to %s\n",id,path.c_str());

	cv::imwrite( path+"_rgb.png", rgb );
	cv::imwrite( path+"_depth.png", depth );

	unsigned long buffersize = 19*sizeof(double);
	char* buffer = new char[buffersize];
	double * buffer_double = (double *)buffer;
	unsigned long * buffer_long = (unsigned long *)buffer;

	int counter = 0;
	buffer_double[counter++] = capturetime;
	for(int i = 0; i < 4; i++){
		for(int j = 0; j < 4; j++){
			buffer_double[counter++] = pose(i,j);
		}
	}
	buffer_long[counter++] = sweepid;
	buffer_long[counter++] = camera->id;
	std::ofstream outfile (path+"_data.txt",std::ofstream::binary);
	outfile.write (buffer,buffersize);
	outfile.close();
	delete[] buffer;
}

RGBDFrame * RGBDFrame::load(Camera * cam, std::string path){
	printf("RGBDFrame * RGBDFrame::load(Camera * cam, std::string path)\n");

	std::streampos size;
	char * buffer;
	char buf [1024];
	std::string datapath = path+"_data.txt";
	std::ifstream file (datapath, std::ios::in | std::ios::binary | std::ios::ate);
	if (file.is_open()){
		size = file.tellg();
		buffer = new char [size];
		file.seekg (0, std::ios::beg);
		file.read (buffer, size);
		file.close();

		double * buffer_double = (double *)buffer;
		unsigned long * buffer_long = (unsigned long *)buffer;

		int counter = 0;
		double capturetime = buffer_double[counter++];
		Eigen::Matrix4d pose;
		for(int i = 0; i < 4; i++){
			for(int j = 0; j < 4; j++){
				pose(i,j) = buffer_double[counter++];
			}
		}
		int sweepid = buffer_long[counter++];
		int camera_id = buffer_long[counter++];

		cv::Mat rgb = cv::imread(path+"_rgb.png", -1);   // Read the file
		cv::Mat depth = cv::imread(path+"_depth.png", -1);   // Read the file

		RGBDFrame * frame = new RGBDFrame(cam,rgb,depth,capturetime,pose);
		frame->sweepid = sweepid;
		//printf("sweepid: %i",sweepid);

		return frame;
	}else{printf("cant open %s\n",(path+"/data.txt").c_str());}
}

}
