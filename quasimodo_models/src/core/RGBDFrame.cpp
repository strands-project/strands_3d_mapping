#include "RGBDFrame.h"

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

//VTK include needed for drawing graph lines
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

#include "seeds3.cpp"
#include "seeds2.cpp"
#include <vector>
#include <string>

#include "seeds2.h"

#include <cv.h>
#include <highgui.h>
#include <fstream>


#include "helper.h"

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
	id = RGBDFrame_id_counter++;
	camera = camera_;
	rgb = rgb_;
	depth = depth_;
	capturetime = capturetime_;
	pose = pose_;

	IplImage iplimg = rgb_;
	IplImage* img = &iplimg;

	int width = img->width;
	int height = img->height;
	int sz = height*width;

	connections.resize(1);
	connections[0].resize(1);
	connections[0][0] = 0;

	intersections.resize(1);
	intersections[0].resize(1);
	intersections[0][0] = 0;

	nr_labels = 1;
	labels = new int[width*height];
	for(unsigned int i = 0; i < width*height; i++){labels[i] = 0;}

	printf("Image loaded %d\n",img->nChannels);




	//unsigned short * oriframe = (unsigned short *)depth.data;
	//for(int i = 10000; i < 640*480; i+=10000){printf("%i depth %i\n",i,oriframe[i]);}
	//exit(0);

	if(compute_normals){
		const unsigned int width	= camera->width;
		const unsigned int height	= camera->height;

		normals.create(height,width,CV_32FC3);
		float * normalsdata = (float *)normals.data;

		unsigned short * depthdata = (unsigned short *)depth.data;
		unsigned char * rgbdata = (unsigned char *)rgb.data;



		const double idepth			= camera->idepth_scale;
		const double cx				= camera->cx;
		const double cy				= camera->cy;
		const double ifx			= 1.0/camera->fx;
		const double ify			= 1.0/camera->fy;

		printf("idepth: %f\n",idepth);

		pcl::PointCloud<pcl::PointXYZ>::Ptr	cloud	(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr	normals (new pcl::PointCloud<pcl::Normal>);
		cloud->width	= width;
		cloud->height	= height;
		cloud->points.resize(width*height);

		for(unsigned int w = 0; w < width; w++){
			for(unsigned int h = 0; h < height;h++){
				int ind = h*width+w;
				pcl::PointXYZ & p = cloud->points[ind];
				//p.b = rgbdata[3*ind+0];
				//p.g = rgbdata[3*ind+1];
				//p.r = rgbdata[3*ind+2];
				double z = idepth*double(depthdata[ind]);
				//if(w % 10 == 0 && h % 10 == 0){printf("%i %i -> %f\n",w,h,z);}

				if(z > 0){
					p.x = (double(w) - cx) * z * ifx;
					p.y = (double(h) - cy) * z * ify;
					p.z = z;
					//cloud->points[ind] = p;
				}else{
					p.x = NAN;
					p.y = NAN;
					p.z = NAN;
				}
			}
		}

		//pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);

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

		for(unsigned int w = 0; w < width; w++){
			for(unsigned int h = 0; h < height;h++){
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
				for(unsigned int w = 0; w < width; w++){
					for(unsigned int h = 0; h < height;h++){
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


				for(unsigned int w = 0; w < width; w++){
					for(unsigned int h = 0; h < height;h++){
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
				sprintf(buf,"combined%i.png",id);
				cv::imwrite( buf, combined );
				printf("saving: %s\n",buf);
				cv::imshow( "combined", combined );
				cv::waitKey(0);
				//while(!updated){cv::waitKey(50);}
				updated = false;
			//}
		}
			//exit(0);
	}
/*
		unsigned int* ubuff = new unsigned int[sz];
		unsigned int* ubuff2 = new unsigned int[sz];
		unsigned int* dbuff = new unsigned int[sz];
		unsigned int pValue;
		unsigned int pdValue;
		char c;
		unsigned int r,g,b,d,dx,dy;
		int idx = 0;
		/// Create Windows
		cv::namedWindow("segoutput_labels", 1);
		cv::namedWindow("segoutput_boundary", 1);

		int seed_width = 3;
		int seed_height = 4;
		int nr_levels = 5;
		int NR_BINS = 5; // Number of bins in each histogram channel
		int normal_NR_BINS = 5; // Number of bins in each histogram channel

		int smoothing_prior_slider = 10;
		int smoothing_power_slider = 40;
		int xydistance_prior_slider = 100;
		int xydistance_power_slider = 40;

		int colorhist_weight_slider = 10;
		int normalhist_weight_slider = 0;

		int nr_iter = 10;

		cv::createTrackbar( "smoothing_prior", "segoutput_labels", &smoothing_prior_slider, 100, on_trackbar );
		cv::createTrackbar( "smoothing_power", "segoutput_labels", &smoothing_power_slider, 100, on_trackbar );
		cv::createTrackbar( "xydistance_prior", "segoutput_labels", &xydistance_prior_slider, 1000, on_trackbar );
		cv::createTrackbar( "xydistance_power", "segoutput_labels", &xydistance_power_slider, 100, on_trackbar );

		cv::createTrackbar( "colorhist_weight", "segoutput_labels", &colorhist_weight_slider, 100, on_trackbar );
		cv::createTrackbar( "normalhist_weight", "segoutput_labels", &normalhist_weight_slider, 100, on_trackbar );
		cv::createTrackbar( "nr_iter", "segoutput_labels", &nr_iter, 200, on_trackbar );
		cv::createTrackbar( "seed_width", "segoutput_labels", &seed_width, 15, on_trackbar );
		cv::createTrackbar( "seed_height", "segoutput_labels", &seed_height, 15, on_trackbar );
		cv::createTrackbar( "nr_levels", "segoutput_labels", &nr_levels, 15, on_trackbar );
		cv::createTrackbar( "NR_BINS", "segoutput_labels", &NR_BINS, 25, on_trackbar );
		sz = 3*width*height;
		unsigned int * output_buff = new unsigned int[sz];
	while(true){


		idx = 0;
			for(int j=0;j<img->height;j++){
			  for(int i=0;i<img->width;i++){
				  if(img->nChannels == 3){
					  // image is assumed to have data in BGR order
					  b = ((uchar*)(img->imageData + img->widthStep*(j)))[(i)*img->nChannels];
					  g = ((uchar*)(img->imageData + img->widthStep*(j)))[(i)*img->nChannels+1];
					  r = ((uchar*)(img->imageData + img->widthStep*(j)))[(i)*img->nChannels+2];
					  if (d < 128) d = 0;
					  pValue = b | (g << 8) | (r << 16);
					}else if(img->nChannels == 1){
					  c = ((uchar*)(img->imageData + img->widthStep*(j)))[(i)*img->nChannels];
					  pValue = c | (c << 8) | (c << 16);
					}else{
					  printf("Unknown number of channels %d\n", img->nChannels);exit(0);
					}
				  ubuff[idx] = pValue;
				  ubuff2[idx] = pValue;
				  idx++;
				}
			}

		SEEDS seeds(width, height, 3, NR_BINS,normal_NR_BINS);
		seeds.smoothing_prior = double(smoothing_prior_slider)*0.1;
		seeds.smoothing_power = double(smoothing_power_slider)*0.1;
		seeds.xydistance_prior = double(xydistance_prior_slider)*1;
		seeds.xydistance_power = double(xydistance_power_slider)*0.1;
		seeds.colorhist_weight = double(colorhist_weight_slider);
		seeds.normalhist_weight = double(normalhist_weight_slider);
		seeds.nr_iter = nr_iter;

		//SEEDS2 seeds(width, height, 3, NR_BINS,normal_NR_BINS);
		//seeds.smoothing_prior = double(smoothing_prior_slider)*0.1;
		//seeds.smoothing_power = double(smoothing_power_slider)*0.1;
		//seeds.xydistance_prior = double(xydistance_prior_slider)*1;
		//seeds.xydistance_power = double(xydistance_power_slider)*0.1;
		//seeds.colorhist_weight = double(colorhist_weight_slider);
		//seeds.normalhist_weight = double(normalhist_weight_slider);
		//seeds.nr_iter = nr_iter;
		seeds.initialize(seed_width, seed_height, nr_levels);
		seeds.update_image_ycbcr(ubuff,normalsdata);
		seeds.iterate();

		printf("done all\n");
		printf("SEEDS produced %d labels\n", seeds.count_superpixels());



		for (int i = 0; i<sz; i++){output_buff[i] = 0;}



		int nr_pixels = seeds.count_superpixels();
		//std::vector< std::vector<double> > connections;
		connections.resize(nr_pixels);
		for (int i = 0; i<nr_pixels; i++){
			connections[i].resize(nr_pixels);
			for (int j = 0; j<nr_pixels; j++){
				connections[i][j] = 0;
			}
		}



		unsigned int * labels = seeds.labels[seeds.seeds_top_level];	 //[level][y * width + x]
		unsigned int * means = seeds.means;

		for (int x=0; x<width-1; x++){
			for (int y=0; y<height; y++){
				int i = y*width +x;
				int li = labels[i];
				int lip = labels[i+1];
				if(li != lip){
					double weight = 1;

					double z1 = idepth*double(depthdata[i]);
					double z2 = idepth*double(depthdata[i+1]);

					//printf("%i %i -> %f %f\n",x,y,z1,z2);
					if(z1 > 0 && z2 > 0){
						if( fabs(z1-z2) < 0.1){
							connections[li][lip]+=weight;
							connections[lip][li]+=weight;
						}
					}
				}
			}
		}

		for (int x=0; x<width; x++){
			for (int y=0; y<height-1; y++){
				int i = y*width +x;
				int li = labels[i];
				int lip = labels[i+width];
				if(li != lip){
					double weight = 1;

					double z1 = idepth*double(depthdata[i]);
					double z2 = idepth*double(depthdata[i+width]);


					if(z1 > 0 && z2 > 0){
						if( fabs(z1-z2) < 0.1){
							//printf("%i %i -> %f %f\n",x,y,z1,z2);
							connections[li][lip]+=weight;
							connections[lip][li]+=weight;
						}
					}
				}
			}
		}

		//std::vector< std::vector<double> > intersections;
		intersections.resize(nr_pixels);
		for (int i = 0; i<nr_pixels; i++){
			intersections[i].resize(nr_pixels);
			for (int j = 0; j<nr_pixels; j++){
				intersections[i][j] = 0;
			}
		}

		for (int i = 0; i<nr_pixels; i++){
			for (int j = 0; j<nr_pixels; j++){
				if(connections[i][j] != 0){
					intersections[i][j] = seeds.intersection(seeds.seeds_top_level, i, seeds.seeds_top_level, j);
					//printf("%i %i -> %f\n",i,j,intersections[i][j]);
				}
			}
		}

		printf("Draw Contours Around Segments\n");
		DrawContoursAroundSegments(ubuff, seeds.labels[nr_levels-1], width, height, 0xff0000, false);//0xff0000 draws red contours
		DrawContoursAroundSegments(output_buff, seeds.labels[nr_levels-1], width, height, 0xffffff, true);//0xff0000 draws white contours

		  SaveImage(ubuff,			width, height,"segoutput_labels.png");
		  SaveImage(output_buff,	width, height,"segoutput_boundary.png");
		  seeds.SaveLabels_Text("segoutput.seg");





		  cv::Mat src1 = cv::imread("segoutput_labels.png");
		  cv::Mat src_graph = cv::imread("segoutput_labels.png");
		  cv::Mat src2 = cv::imread("segoutput_boundary.png");

		  for(unsigned int w = 0; w < width; w++){
			  for(unsigned int h = 0; h < height;h++){
				  int ind = h*width+w;
				  pcl::Normal		p2		= normals->points[ind];
				  if(!isnan(p2.normal_x)){
					cv::circle(src_graph, cv::Point(w,h), 1, cv::Scalar(255,0,255));
				  }
			  }
		  }

			for (int i = 0; i<nr_pixels; i++){
				float px = seeds.X_channel[i] / seeds.T[seeds.seeds_top_level][i];
				float py = seeds.Y_channel[i] / seeds.T[seeds.seeds_top_level][i];
				cv::circle(src_graph, cv::Point(px,py), 5, cv::Scalar(0,255,0));
				for (int j = 0; j<nr_pixels; j++){
					if(connections[i][j] != 0){
						float pxj = seeds.X_channel[j] / seeds.T[seeds.seeds_top_level][j];
						float pyj = seeds.Y_channel[j] / seeds.T[seeds.seeds_top_level][j];
						float score = intersections[i][j];
						cv::line(src_graph, cv::Point(px,py), cv::Point(pxj,pyj), cv::Scalar(255.0*(1-score),0,255.0*(score)),1);
					}
				}
			}





		  cv::imshow( "segoutput_labels", src1 );
		  cv::imshow( "segoutput_graph", src_graph );
		  cv::imshow( "segoutput_boundary", src2 );
		  cv::waitKey(0);
		  printf("Done!\n");

	}

		delete[] ubuff;
		delete[] output_buff;
	exit(0);
	*/
	//}

	//show(true);
}

RGBDFrame::~RGBDFrame(){}

void RGBDFrame::show(bool stop){
	cv::namedWindow( "rgb", cv::WINDOW_AUTOSIZE );		cv::imshow( "rgb", rgb );
	cv::namedWindow( "normals", cv::WINDOW_AUTOSIZE );	cv::imshow( "normals", normals );
	cv::namedWindow( "depth", cv::WINDOW_AUTOSIZE );	cv::imshow( "depth", depth );
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

}
