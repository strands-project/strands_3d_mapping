
#include "Transformation.h"
#include <string>
#include <iostream>

#include <Eigen/Core>
#include <fstream>

Transformation * Transformation::clone(){
	Transformation * transformation = new Transformation();
	transformation->transformationMatrix = transformationMatrix;
	transformation->src		= src;
	transformation->dst		= dst;
	transformation->weight	= weight;
	transformation->time	= time;
	for(unsigned int i = 0; i < matches.size();i++){
		KeyPoint * src_kp = matches.at(i).first;
		KeyPoint * dst_kp = matches.at(i).second;
		transformation->matches.push_back(make_pair(src_kp, dst_kp));
	}

	for(unsigned int i = 0; i < plane_matches.size();i++){
		Plane * src_kp = plane_matches.at(i).first;
		Plane * dst_kp = plane_matches.at(i).second;
		transformation->plane_matches.push_back(make_pair(src_kp, dst_kp));
	}
	//time = -1;
	
	return transformation;
}

void setPCDcolor(float r, float g, float b, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
	for(unsigned int i = 0; i < cloud->points.size(); i++){
		cloud->points.at(i).r = r;
		cloud->points.at(i).g = g;
		cloud->points.at(i).b = b;
	}
}

void setPCDcolor(float r, float g, float b, pcl::PointCloud<pcl::PointXYZRGB> cloud){
	printf("setPCDcolor(%f %f %f)\n",r,g,b);
	for(unsigned int i = 0; i < cloud.points.size(); i++){
		cloud.points.at(i).r = r;
		cloud.points.at(i).g = g;
		cloud.points.at(i).b = b;
	}
}

void addToPCD(float r, float g, float b, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2, Matrix4f trans){
	pcl::PointCloud<pcl::PointXYZRGB> cloud_trans;
	pcl::transformPointCloud (*cloud2, cloud_trans, trans);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	*tmp_cloud = cloud_trans;

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setLeafSize (0.001f, 0.001f, 0.001f);
	sor.setInputCloud (tmp_cloud);


	pcl::PointCloud<pcl::PointXYZRGB> voxelcloud_trans;
	sor.filter(voxelcloud_trans);

	for(unsigned int i = 0; i < voxelcloud_trans.points.size(); i++){
		voxelcloud_trans.points.at(i).r = r;
		voxelcloud_trans.points.at(i).g = g;
		voxelcloud_trans.points.at(i).b = b;
	}
	*cloud1 += voxelcloud_trans;
}

void Transformation::show(bool pcd){

	IplImage * dummy = getMatchImg();//cvCreateImage(cvSize(100, 100), IPL_DEPTH_8U, 3);
	cvShowImage("dummy", dummy);

	if(pcd){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		*src_cloud = src->input->getCloud();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr dst_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		*dst_cloud = dst->input->getCloud();

		Eigen::Matrix4f src_mat = src->input->gt.matrix();
		Eigen::Matrix4f dst_mat = dst->input->gt.matrix();

		float scaleval = 1;
		Eigen::Matrix4f scale = Eigen::Matrix4f::Identity()*scaleval;


		Eigen::Matrix4f true_mat = src_mat.inverse() * dst_mat;
		Eigen::Matrix4f trans_mat = transformationMatrix.inverse();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

		addToPCD(255,0	,0,		cloud, src_cloud, Eigen::Matrix4f::Identity());
		addToPCD(0,255,0,		cloud, dst_cloud, trans_mat);
	

		if(!viewer){
			viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
			viewer->setBackgroundColor (0, 0, 0);
			viewer->initCameraParameters ();
		}
		if(viewer->wasStopped ()){
			viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
			viewer->setBackgroundColor (0, 0, 0);
			viewer->initCameraParameters ();
		}

		viewer->removePointCloud("sample cloud");
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.1, "sample cloud");
		viewer->addCoordinateSystem (scaleval*0.2);
		while (!viewer->wasStopped () && cvWaitKey(10) == -1){
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
		viewer->close();
	}else{cvWaitKey(0);}
	cvReleaseImage( &dummy);

}
void Transformation::show(pcl::visualization::CloudViewer * viewer){};
void Transformation::print(){
	//printf("%i %i -> transformation: %s -> %s , weight: %f\n",src->id,dst->id,src->input->rgb_path.c_str(),dst->input->rgb_path.c_str(),weight);
	printf("%i %i weight: %f\n",src->id,dst->id,weight);
}

IplImage * Transformation::getMatchImg(){
	IplImage* rgb_img_src 	= src->input->rgb_img;//cvLoadImage(src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	char * data_src = (char *)rgb_img_src->imageData;
	unsigned short * depth_data_src		= (unsigned short *)(src->input->depth_img->imageData);

	IplImage* rgb_img_dst 	= dst->input->rgb_img;//cvLoadImage(dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	char * data_dst = (char *)rgb_img_dst->imageData;
	unsigned short * depth_data_dst		= (unsigned short *)(dst->input->depth_img->imageData);

	int	width = rgb_img_src->width;
	int	height = rgb_img_src->height;

	IplImage* img_combine = cvCreateImage(cvSize(2*width,height), IPL_DEPTH_8U, 3);
	char * data = (char *)img_combine->imageData;

	for (int j = 0; j < height; j++){
		for (int i = 0; i < width; i++){
			int ind = 3*(640*j+i);
			int dst_ind = 3 * (j * (2*width) + (width+i));
			int src_ind = 3 * (j * (2*width) + (i));
			int d_dst = depth_data_dst[(640*j+i)];
			int d_src = depth_data_src[(640*j+i)];
			if(d_dst == 0 && (i % 2 == 0) && (j % 2 == 0)){
				data[dst_ind + 0] = 255;
				data[dst_ind + 1] = 0;
				data[dst_ind + 2] = 255;
			}else{
				data[dst_ind + 0] = data_dst[ind +0];
				data[dst_ind + 1] = data_dst[ind +1];
				data[dst_ind + 2] = data_dst[ind +2];
			}
			if(d_src == 0 && (i % 2 == 0) && (j % 2 == 0)){
				data[src_ind + 0] = 255;
				data[src_ind + 1] = 0;
				data[src_ind + 2] = 255;
			}else{
				data[src_ind + 0] = data_src[ind +0];
				data[src_ind + 1] = data_src[ind +1];
				data[src_ind + 2] = data_src[ind +2];
			}
		}
	}
	//cvReleaseImage( &rgb_img_src );
	//cvReleaseImage( &rgb_img_dst );

	for(unsigned int i = 0; i < src->keypoints->valid_key_points.size(); i++){
		KeyPoint * src_kp = src->keypoints->valid_key_points.at(i);
		cvCircle(img_combine,cvPoint(src_kp->point->w			, src_kp->point->h), 3,cvScalar(0, 255, 0, 0),1, 8, 0);
	}

	for(unsigned int i = 0; i < src->keypoints->invalid_key_points.size(); i++){
		KeyPoint * kp = src->keypoints->invalid_key_points.at(i);
		cvCircle(img_combine,cvPoint(kp->point->w			, kp->point->h), 3,cvScalar(0, 255, 255, 0),1, 8, 0);
	}

	for(unsigned int i = 0; i < dst->keypoints->valid_key_points.size(); i++){
		KeyPoint * dst_kp = dst->keypoints->valid_key_points.at(i);
		cvCircle(img_combine,cvPoint(dst_kp->point->w + width	, dst_kp->point->h), 3,cvScalar(0, 255, 0, 0),1, 8, 0);
	}

	for(unsigned int i = 0; i < dst->keypoints->invalid_key_points.size(); i++){
		KeyPoint * kp = dst->keypoints->invalid_key_points.at(i);
		cvCircle(img_combine,cvPoint(kp->point->w + width	, kp->point->h), 3,cvScalar(0, 255, 255, 0),1, 8, 0);
	}

	for(unsigned int i = 0; i < matches.size(); i++){
		KeyPoint * src_kp = matches.at(i).first;
		KeyPoint * dst_kp = matches.at(i).second;
		cvLine(img_combine,cvPoint(dst_kp->point->w  + width ,dst_kp->point->h),cvPoint(src_kp->point->w,src_kp->point->h),cvScalar(0, 0, 255, 0),1, 8, 0);
	}

	return img_combine;
}

void Transformation::saveProblem(string path){
	printf("saveProblem(%s)\n",path.c_str());
	IplImage* rgb_img_src 	= cvLoadImage(src->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	char * data_src = (char *)rgb_img_src->imageData;
	unsigned short * depth_data_src		= (unsigned short *)(src->input->depth_img->imageData);

	IplImage* rgb_img_dst 	= cvLoadImage(dst->input->rgb_path.c_str(),CV_LOAD_IMAGE_UNCHANGED);
	char * data_dst = (char *)rgb_img_dst->imageData;
	unsigned short * depth_data_dst		= (unsigned short *)(dst->input->depth_img->imageData);

	int	width = rgb_img_src->width;
	int	height = rgb_img_src->height;

	IplImage* img_combine = cvCreateImage(cvSize(2*width,height), IPL_DEPTH_8U, 3);
	char * data = (char *)img_combine->imageData;

	for (int j = 0; j < height; j++){
		for (int i = 0; i < width; i++){
			int ind = 3*(640*j+i);
			int dst_ind = 3 * (j * (2*width) + (width+i));
			int src_ind = 3 * (j * (2*width) + (i));
			int d_dst = depth_data_dst[(640*j+i)];
			int d_src = depth_data_src[(640*j+i)];
			if(d_dst == 0 && (i % 2 == 0) && (j % 2 == 0)){
				data[dst_ind + 0] = 255;
				data[dst_ind + 1] = 0;
				data[dst_ind + 2] = 255;
			}else{
				data[dst_ind + 0] = data_dst[ind +0];
				data[dst_ind + 1] = data_dst[ind +1];
				data[dst_ind + 2] = data_dst[ind +2];
			}
			if(d_src == 0 && (i % 2 == 0) && (j % 2 == 0)){
				data[src_ind + 0] = 255;
				data[src_ind + 1] = 0;
				data[src_ind + 2] = 255;
			}else{
				data[src_ind + 0] = data_src[ind +0];
				data[src_ind + 1] = data_src[ind +1];
				data[src_ind + 2] = data_src[ind +2];
			}
		}
	}
	cvReleaseImage( &rgb_img_src );
	cvReleaseImage( &rgb_img_dst );

	for(unsigned int i = 0; i < src->keypoints->valid_key_points.size(); i++){
		KeyPoint * src_kp = src->keypoints->valid_key_points.at(i);
		cvCircle(img_combine,cvPoint(src_kp->point->w			, src_kp->point->h), 3,cvScalar(0, 255, 0, 0),1, 8, 0);
	}

	for(unsigned int i = 0; i < src->keypoints->invalid_key_points.size(); i++){
		KeyPoint * kp = src->keypoints->invalid_key_points.at(i);
		cvCircle(img_combine,cvPoint(kp->point->w			, kp->point->h), 3,cvScalar(0, 255, 255, 0),1, 8, 0);
	}

	for(unsigned int i = 0; i < dst->keypoints->valid_key_points.size(); i++){
		KeyPoint * dst_kp = dst->keypoints->valid_key_points.at(i);
		cvCircle(img_combine,cvPoint(dst_kp->point->w + width	, dst_kp->point->h), 3,cvScalar(0, 255, 0, 0),1, 8, 0);
	}

	for(unsigned int i = 0; i < dst->keypoints->invalid_key_points.size(); i++){
		KeyPoint * kp = dst->keypoints->invalid_key_points.at(i);
		cvCircle(img_combine,cvPoint(kp->point->w + width	, kp->point->h), 3,cvScalar(0, 255, 255, 0),1, 8, 0);
	}

	for(unsigned int i = 0; i < matches.size() ; i++){
		KeyPoint * src_kp = matches.at(i).first;
		KeyPoint * dst_kp = matches.at(i).second;
		cvLine(img_combine,cvPoint(dst_kp->point->w  + width ,dst_kp->point->h),cvPoint(src_kp->point->w,src_kp->point->h),cvScalar(0, 0, 255, 0),1, 8, 0);
	}

	cvShowImage("combined image", img_combine);
	char buf[1024];
	sprintf(buf,"%i_%s.png",(int)matches.size(),path.c_str());
	if(!cvSaveImage(buf,img_combine)){printf("Could not save: %s\n",buf);}
	cvWaitKey(30);
	cvReleaseImage( &img_combine);
}
