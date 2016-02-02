#include "Model.h"

namespace reglib
{

unsigned int model_id_counter = 0;

Model::Model(){id = model_id_counter++;}

Model::Model(RGBDFrame * frame, cv::Mat mask, Eigen::Matrix4d pose){
	id = model_id_counter++;
	relativeposes.push_back(pose);
	frames.push_back(frame);
	masks.push_back(mask);
}

Model::~Model(){}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Model::getPCLcloud(int step, bool color){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	for(unsigned int i = 0; i < frames.size(); i++){
		cv::Mat rgb = frames[i]->rgb;
		unsigned char * rgbdata = (unsigned char *)rgb.data;

		cv::Mat depth = frames[i]->depth;
		unsigned short * depthdata = (unsigned short *)depth.data;

		Camera * camera = frames[i]->camera;
		const unsigned int width = camera->width;
		const unsigned int height = camera->height;
		const double idepth = camera->idepth_scale;
		const double cx		= camera->cx;
		const double cy		= camera->cy;
		const double ifx	= 1.0/camera->fx;
		const double ify	= 1.0/camera->fy;

		Eigen::Matrix4d pose = relativeposes[i];
		const double m00 = pose(0,0); const double m01 = pose(0,1); const double m02 = pose(0,2); const double m03 = pose(0,3);
		const double m10 = pose(1,0); const double m11 = pose(1,1); const double m12 = pose(1,2); const double m13 = pose(1,3);
		const double m20 = pose(2,0); const double m21 = pose(2,1); const double m22 = pose(2,2); const double m23 = pose(2,3);

		int r = (rand()%4)*255/4; int g = (rand()%4)*255/4; int b = (rand()%4)*255/4;

		for(unsigned int w = 0; w < width; w+=step){
			for(unsigned int h = 0; h < height;h+=step){
				int ind = h*width+w;
				double z = idepth*double(depthdata[ind]);
				if(z > 0){
					pcl::PointXYZRGB p;
					double x = (double(w) - cx) * z * ifx;
					double y = (double(h) - cy) * z * ify;
					p.x = m00*x + m01*y + m02*z + m03;
					p.y = m10*x + m11*y + m12*z + m13;
					p.z = m20*x + m21*y + m22*z + m23;
					if(color){
						p.b = rgbdata[3*ind+0];
						p.g = rgbdata[3*ind+1];
						p.r = rgbdata[3*ind+2];
					}else{
						p.b = r;
						p.g = g;
						p.r = b;
					}
					cloud_ptr->points.push_back(p);
				}
			}
		}
	}
	return cloud_ptr;
}

}

