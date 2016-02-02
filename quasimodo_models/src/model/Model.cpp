#include "Model.h"

namespace reglib
{

unsigned int model_id_counter = 0;

Model::Model(){
    id = model_id_counter++;
}

using namespace Eigen;

Model::Model(RGBDFrame * frame, cv::Mat mask, Eigen::Matrix4d pose){
//printf("start Model::Model(RGBDFrame * frame, cv::Mat mask, Eigen::Matrix4d pose)\n");

    id = model_id_counter++;

//	addFrameToModel(frame, mask, pose);
	unsigned char  * maskdata		= (unsigned char	*)(mask.data);
	unsigned char  * rgbdata		= (unsigned char	*)(frame->rgb.data);
	unsigned short * depthdata		= (unsigned short	*)(frame->depth.data);
	float		   * normalsdata	= (float			*)(frame->normals.data);

	unsigned int frameid = frame->id;

	Camera * camera				= frame->camera;
	const unsigned int width	= camera->width;
	const unsigned int height	= camera->height;
	const unsigned int width2	= width-2;
	const unsigned int height2	= height-2;
	const float idepth			= camera->idepth_scale;
	const float cx				= camera->cx;
	const float cy				= camera->cy;
	const float ifx				= 1.0/camera->fx;
	const float ify				= 1.0/camera->fy;
	const float fx				= camera->fx;
	const float fy				= camera->fy;

//HACK, should only be done if pose == identity
	//std::vector<superpoint> pointsToAdd;
	points.reserve(width*height);
	for(unsigned int w = 0; w < width; w+=1){
		for(unsigned int h = 0; h < height;h+=1){
			int ind = h*width+w;
			if(maskdata[ind] == 255){// && p.z > 0 && !isnan(p.normal_x)){
				float z = idepth*float(depthdata[ind]);
				float nx = normalsdata[3*ind+0];

				if(z > 0 && nx != 2){
					float ny = normalsdata[3*ind+1];
					float nz = normalsdata[3*ind+2];

					float x = (w - cx) * z * ifx;
					float y = (h - cy) * z * ify;

					float pb = rgbdata[3*ind+0];
					float pg = rgbdata[3*ind+1];
					float pr = rgbdata[3*ind+2];

					Vector3f	pxyz	(x	,y	,z );
					Vector3f	pnxyz	(nx,ny,nz);
					Vector3f	prgb	(pr	,pg	,pb );
					float		weight	= 1.0/(z*z);
					points.push_back(superpoint(pxyz,pnxyz,prgb, weight, weight, frameid));
				}
			}
		}
	}

	relativeposes.push_back(pose);
	frames.push_back(frame);
	masks.push_back(mask);
	
	char buf [1024];
//	sprintf(buf,"Frame %i maskimage",frame->id);
//	cv::namedWindow(buf,	cv::WINDOW_AUTOSIZE);
//	cv::imshow(		buf,	mask);

//	sprintf(buf,"Frame %i frame->depth",frame->id);
//	cv::namedWindow(buf,	cv::WINDOW_AUTOSIZE);
//	cv::imshow(		buf,	frame->depth);
	
//	sprintf(buf,"Frame %i frame->rgb",frame->id);
//	cv::namedWindow(buf,	cv::WINDOW_AUTOSIZE);
//	cv::imshow(		buf,	frame->rgb);

	cv::waitKey(100);
//printf("end Model::Model(RGBDFrame * frame, cv::Mat mask, Eigen::Matrix4d pose)\n");
}

void Model::recomputeModelPoints(){
	points.clear();
	for(int i = 0; i < frames.size(); i++){
		addPointsToModel(frames[i],masks[i],relativeposes[i]);
	}
}

void Model::addPointsToModel(RGBDFrame * frame, cv::Mat mask, Eigen::Matrix4d p){
	unsigned char  * maskdata		= (unsigned char	*)(mask.data);
	unsigned char  * rgbdata		= (unsigned char	*)(frame->rgb.data);
	unsigned short * depthdata		= (unsigned short	*)(frame->depth.data);
	float		   * normalsdata	= (float			*)(frame->normals.data);

	unsigned int frameid = frame->id;

	float m00 = p(0,0); float m01 = p(0,1); float m02 = p(0,2); float m03 = p(0,3);
	float m10 = p(1,0); float m11 = p(1,1); float m12 = p(1,2); float m13 = p(1,3);
	float m20 = p(2,0); float m21 = p(2,1); float m22 = p(2,2); float m23 = p(2,3);

	Camera * camera				= frame->camera;
	const unsigned int width	= camera->width;
	const unsigned int height	= camera->height;
	const float idepth			= camera->idepth_scale;
	const float cx				= camera->cx;
	const float cy				= camera->cy;
	const float ifx				= 1.0/camera->fx;
	const float ify				= 1.0/camera->fy;

	bool reprojected [width*height];
	for(unsigned int i = 0; i < width*height; i++){reprojected[i] = false;}

	//std::vector<superpoint> pointsToAdd;
	for(unsigned int w = 0; w < width; w++){
		for(unsigned int h = 0; h < height;h++){
			int ind = h*width+w;
			if(maskdata[ind] == 255 && !reprojected[ind]){// && p.z > 0 && !isnan(p.normal_x)){
				float z = idepth*float(depthdata[ind]);
				float nx = normalsdata[3*ind+0];

				if(z > 0 && nx != 2){
					float ny = normalsdata[3*ind+1];
					float nz = normalsdata[3*ind+2];

					float x = (w - cx) * z * ifx;
					float y = (h - cy) * z * ify;

					float px	= m00*x + m01*y + m02*z + m03;
					float py	= m10*x + m11*y + m12*z + m13;
					float pz	= m20*x + m21*y + m22*z + m23;
					float pnx	= m00*nx + m01*ny + m02*nz;
					float pny	= m10*nx + m11*ny + m12*nz;
					float pnz	= m20*nx + m21*ny + m22*nz;

					float pb = rgbdata[3*ind+0];
					float pg = rgbdata[3*ind+1];
					float pr = rgbdata[3*ind+2];

					Vector3f	pxyz	(px	,py	,pz );
					Vector3f	pnxyz	(pnx,pny,pnz);
					Vector3f	prgb	(pr	,pg	,pb );
					float		weight	= 1.0/(z*z);
					points.push_back(superpoint(pxyz,pnxyz,prgb, weight, weight, frameid));
				}
			}
		}
	}
}

void Model::addFrameToModel(RGBDFrame * frame, cv::Mat mask, Eigen::Matrix4d p){
	addPointsToModel(frame, mask, p);
	
	relativeposes.push_back(p);
	frames.push_back(frame);
	masks.push_back(mask);
}

CloudData * Model::getCD(unsigned int target_points){
	std::vector<unsigned int> ro;
	unsigned int nc = points.size();
	ro.resize(nc);
	for(unsigned int i = 0; i < nc; i++){ro[i] = i;}
	for(unsigned int i = 0; i < nc; i++){
		unsigned int randval = rand();
		unsigned int rind = randval%nc;
		int tmp = ro[i];
		ro[i] = ro[rind];
		ro[rind] = tmp;
	}
	//Build registration input
	unsigned int nr_points = std::min(unsigned(points.size()),target_points);
	MatrixXd data			(6,nr_points);
	MatrixXd data_normals	(3,nr_points);
	MatrixXd information	(6,nr_points);

	for(unsigned int k = 0; k < nr_points; k++){
		superpoint & p		= points[ro[k]];
		data(0,k)			= p.point(0);
		data(1,k)			= p.point(1);
		data(2,k)			= p.point(2);
		data(3,k)			= p.feature(0);
		data(4,k)			= p.feature(1);
		data(5,k)			= p.feature(2);
		data_normals(0,k)	= p.normal(0);
		data_normals(1,k)	= p.normal(1);
		data_normals(2,k)	= p.normal(2);
		information(0,k)	= p.point_information;
		information(1,k)	= p.point_information;
		information(2,k)	= p.point_information;
		information(3,k)	= p.feature_information;
		information(4,k)	= p.feature_information;
		information(5,k)	= p.feature_information;
	}

	CloudData * cd			= new CloudData();
	cd->data				= data;
	cd->information			= information;
	cd->normals				= data_normals;
	return cd;
}

Model::~Model(){}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Model::getPCLcloud(int step, bool color){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	for(unsigned int i = 0; i < points.size(); i+=step){
		superpoint & sp = points[i];
		pcl::PointXYZRGB p;
		p.x = sp.point(0);
		p.y = sp.point(1);
		p.z = sp.point(2);
		if(color){
			p.b =   0;
			p.g = 255;
			p.r =   0;
		}else{
			p.r = sp.feature(0);
			p.g = sp.feature(1);
			p.b = sp.feature(2);
		}
		cloud_ptr->points.push_back(p);
	}
/*
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
*/
	return cloud_ptr;
}

}

