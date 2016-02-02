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

#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;

void do_path(string path, int depth){
	//printf("do_path( %s )\n",path.c_str());
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir (path.c_str())) != NULL) {
		while ((ent = readdir (dir)) != NULL) {
			string filename = ent->d_name;
			if(filename.front() != '.'){

				if(ent->d_type == DT_DIR){
					do_path(path+"/"+filename,depth+1);
				}else if(filename.size() > 4){
					string ending = filename.substr(filename.size()-4,filename.size()-1);
					if(ending.compare(".ply") == 0){
						//for(unsigned int d = 0; d < depth; d++){printf("    ");}
						//printf ("%s\n", filename.c_str());
						string command2run = "pcl_ply2pcd "+path+"/"+filename+" "+path+"/"+filename.substr(0,filename.size()-4)+".pcd";
						system(command2run.c_str());
						printf("command2run: %s\n",command2run.c_str());

						pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
						if (pcl::io::loadPCDFile<pcl::PointXYZ> (path+"/"+filename.substr(0,filename.size()-4)+".pcd", *cloud) != -1){
							clouds.push_back(cloud);

							int width = cloud->width;
							int height = cloud->height;

							pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGB>);
							cloud2->width = width;
							cloud2->height = height;
							cloud2->points.resize(width*height);
							for(int w = 0; w < width; w++){
								for(int h = 0; h < height; h++){
									int ind = h*width+w;

							//for(int w = 0; w < height; w++){
							//	for(int h = 0; h < width; h++){
							//		int ind = h*height+w;
									pcl::PointXYZ p0 = cloud->points[ind];
									pcl::PointXYZRGB & p1 = cloud2->points[ind];
									p1.x = p0.x;
									p1.y = p0.y;
									p1.z = p0.z;
									//if(w > 100 && w < 150){
									if(w % 100 < 50){
										p1.r = 255;
										p1.g = 0;
										p1.b = 0;
									}else{
										p1.r = 0;
										p1.g = 255;
										p1.b = 0;
									}
									//cloud2->points[ind].push_back(p1);
								}
							}

							boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
							viewer->setBackgroundColor (0.5, 0.5, 0.5);
							pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud2);
							viewer->addPointCloud<pcl::PointXYZRGB> (cloud2, rgb, "sample cloud");
							viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
							viewer->addCoordinateSystem (1.0);
							viewer->initCameraParameters ();
							viewer->spin();


							for(int w = 0; w < width-1; w+=20){
								for(int h = 0; h < height-1; h+=20){
									pcl::PointXYZ p0 = cloud->points[h*width+w];

									if(!isnan(p0.x)){

										for(unsigned int w2 = 0; w2 < width; w2+=1){
											p0 = cloud->points[h*width+w2];
											float x0,y0,z0;

											x0 = p0.y;
											y0 = p0.z;
											z0 = p0.x;
											printf("%4.4f %4.4f || ",x0/z0,y0/z0);

											x0 = p0.x;
											y0 = p0.z;
											z0 = p0.y;
											printf("%4.4f %4.4f || ",x0/z0,y0/z0);

											x0 = p0.x;
											y0 = p0.y;
											z0 = p0.z;
											printf("%4.4f %4.4f",x0/z0,y0/z0);
											printf("\n");
										}
										exit(0);
										/*
										printf("===================================================================\n");
										float x0,x1,y0,y1,z0,z1;
										x0 = p0.x; x1 = p1.x;
											y0 = p0.y; y1 = p1.y;
											z0 = p0.z; z1 = p1.z;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);

											y0 = p0.z; y1 = p1.z;
											z0 = p0.y; z1 = p1.y;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);

										x0 = p0.y; x1 = p1.y;
											y0 = p0.x; y1 = p1.x;
											z0 = p0.z; z1 = p1.z;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);

											y0 = p0.z; y1 = p1.z;
											z0 = p0.x; z1 = p1.x;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);


										x0 = p0.z; x1 = p1.z;
											y0 = p0.x; y1 = p1.x;
											z0 = p0.y; z1 = p1.y;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);

											y0 = p0.y; y1 = p1.y;
											z0 = p0.x; z1 = p1.x;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);
										*/
									}
									/*
									pcl::PointXYZ p1 = cloud->points[(h+0)*width+(w+1)];

									if(!isnan(p0.x) && !isnan(p1.x) && (p0.x != 0) && (p1.x != 0)  && (p0.y != 0) && (p1.y != 0) ){
										printf("===================================================================\n");
										float x0,x1,y0,y1,z0,z1;
										x0 = p0.x; x1 = p1.x;
											y0 = p0.y; y1 = p1.y;
											z0 = p0.z; z1 = p1.z;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);

											y0 = p0.z; y1 = p1.z;
											z0 = p0.y; z1 = p1.y;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);

										x0 = p0.y; x1 = p1.y;
											y0 = p0.x; y1 = p1.x;
											z0 = p0.z; z1 = p1.z;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);

											y0 = p0.z; y1 = p1.z;
											z0 = p0.x; z1 = p1.x;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);


										x0 = p0.z; x1 = p1.z;
											y0 = p0.x; y1 = p1.x;
											z0 = p0.y; z1 = p1.y;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);

											y0 = p0.y; y1 = p1.y;
											z0 = p0.x; z1 = p1.x;
											printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);
									}
									*/
								}
							}


						}
						std::cout << "Loaded " << cloud->width * cloud->height << " data points from test_pcd.pcd with the following fields: " << std::endl;

					}
				}
			}
		}
		closedir (dir);
	}
}

int main(int argc, char** argv){

	string data_path = "";
	if(argc > 1){
		clouds.clear();
		for(unsigned int arg = 1; arg < argc; arg++){
			do_path(argv[arg],0);
		}

		printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");

		for(unsigned int c = 0; c < clouds.size(); c++){
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = clouds[c];
			int width = cloud->width;
			int height = cloud->height;
			for(int w = 0; w < width-1; w+=20){
				for(int h = 0; h < height-1; h+=20){
					pcl::PointXYZ p0 = cloud->points[h*width+w];
					pcl::PointXYZ p1 = cloud->points[(h+0)*width+(w+1)];
					if(!isnan(p0.x) && !isnan(p1.x) && (p0.x != 0) && (p1.x != 0)  && (p0.y != 0) && (p1.y != 0) ){
						float x0 = p0.y;
						float y0 = p0.x;
						float z0 = p0.z;

						float x1 = p1.y;
						float y1 = p1.x;
						float z1 = p1.z;

						float a0 = x0/z0;
						float a1 = x1/z1;

						float b0 = w;
						float b1 = w+1;

						//float c0 = y0/z0;
						//float c1 = y1/z1;

						//float d0 = h;
						//float d1 = h+1;

						float fx = (b0-b1)/(a0-a1);
						//float cx = 0;//b0 - fx*a0;

						//float fy = 0;//(d0-d1)/(c0-c1);
						//float cy = 0;//d0 - fy*c0;

						//printf("%i %i -> f: %f %f c: %f\n",w,h,fx,fy,cx,cy);
						printf("%i %i -> p0: %4.4f %4.4f %4.4f p1: %4.4f %4.4f %4.4f \n",w,h,x0/z0,y0/z0,z0/z0,x1/z1,y1/z1,z1/z1);
					}
					//x = (w - cx) * z / fx;

					//fx*a0				= b0 - cx;
					//fx*a0 -b0			= - cx;
					//b0 -fx*a0			= cx;
					//b1 -fx*a1			= cx;
					//b0 -fx*a0			= b1 - fx*a1;
					//b0 -b1 - fx*a0	=  - fx*a1;
					//b0 -b1			= fx*a0 - fx*a1;
					//b0 -b1			= (a0 - a1) * fx;
					//fx				= (b0-b1)/(a0-a1);






					//b0 - fx*a0 = cx;
					//b1 - fx*a1 = cx;

					//b0-b1 = fx*(a0 - a1);
					//fx = (b0-b1)/(a0-a1);
					//cx = b0 - fx*a0;
					//fx*a0 = b0 - cx;
					//b0 = fx*a0 + cx;
					//b0 = fx*a0 + cx;
					//fx*a0 = b0 - cx;
					//fx*a1 = b1 - cx;

					//fx = (b0 - cx)/a0
					//x/z = (w - cx) * ifx;
					//fx*x/z = (w - cx);
					//fx*x/z = ((w+1.0) - cx);
					//float z = idepth*float(depthdata[ind]);
					//float x = (w - cx) * z * ifx;
					//float y = (h - cy) * z * ify;

					//printf("%i %i -> %f %f %f\n",w,h,p.x,p.y,p.z);
				}
			}

			int midw = width/2;
			int midh = height/2;
			printf("%i %i\n",width,height);

			pcl::PointXYZ p00 = cloud->points[midh*width+midw];
			pcl::PointXYZ p10 = cloud->points[midh*width+midw + width];
			pcl::PointXYZ p01 = cloud->points[midh*width+midw + 1];
			pcl::PointXYZ p11 = cloud->points[midh*width+midw + width + 1];
			printf("%i %i -> %f %f %f\n",midw	,midh	,p00.x,p00.y,p00.z);
			printf("%i %i -> %f %f %f\n",midw	,midh+1	,p10.x,p10.y,p10.z);
			printf("%i %i -> %f %f %f\n",midw+1	,midh	,p01.x,p01.y,p01.z);
			printf("%i %i -> %f %f %f\n",midw+1	,midh+1	,p11.x,p11.y,p11.z);
exit(0);
		}
	}
}
