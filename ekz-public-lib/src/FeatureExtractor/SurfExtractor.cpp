#include "SurfExtractor.h"
#include "SurfFeatureDescriptor128.h"
#include <ctime>
#include <algorithm>
#include "cv.h"
#include "highgui.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/gpu/gpu.hpp>
// opencv nonfree includes
//#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/nonfree/gpu.hpp>
//#include "opencv2/ocl/ocl.hpp"
//#include "opencv2/nonfree/nonfree.hpp"

using namespace std;
bool comparison_Surf (KeyPoint * i,KeyPoint * j) { return (i->stabilety>j->stabilety); }

SurfExtractor::SurfExtractor(){
	hessianThreshold = 300;
	nOctaves=4;
	nOctaveLayers=2;
	extended=true;
	upright=false;

	gpu = false;
}
SurfExtractor::~SurfExtractor(){}

KeyPointSet * SurfExtractor::getKeyPointSet(FrameInput * fi){

	struct timeval start, end;
	gettimeofday(&start, NULL);
	KeyPointSet * keypoints = new KeyPointSet();
/*
	IplImage * rgb_img = fi->get_rgb_img();
	cv::Mat img(rgb_img);
	
	cv::Mat desc1;
	vector <cv::KeyPoint> kp1;

	if(!gpu){
		cv::SURF surf = cv::SURF(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);

		cv::Mat gray_img;
		cvtColor(img,gray_img,CV_RGB2GRAY);
		surf(gray_img, cv::Mat(), kp1, desc1);
	}else{
		cv::gpu::GpuMat gpuImg;
		cv::gpu::GpuMat gpuDesc1;
		cv::gpu::GpuMat gpuKp1;

		gpuImg.upload(img);

		cv::gpu::SURF_GPU surfGPU(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);
		surfGPU(gpuImg, cv::gpu::GpuMat(), gpuKp1, gpuDesc1);
		surfGPU.downloadKeypoints(gpuKp1, kp1);
		vector<float> desc_vec;
		surfGPU.downloadDescriptors(gpuDesc1, desc_vec);
		printf("desc_vec.size() = %i\n",(int)desc_vec.size());
		exit(0);
	}

	for(int i = 0; i < (int)kp1.size();i++){
		cv::KeyPoint curr = kp1.at(i);

		int w = curr.pt.x+0.5;
		int h = curr.pt.y+0.5;

		float r,g,b,x,y,z;
		fi->getRGB(r,g,b,curr.pt.x,curr.pt.y);
		fi->getXYZ(x,y,z,curr.pt.x,curr.pt.y);
		
		KeyPoint * kp = new KeyPoint();
		kp->w = curr.pt.x;
		kp->h = curr.pt.y;
		kp->stabilety = curr.response;
		FeatureDescriptor * descriptor = 0;

		if(extended){
			float * desc = new float[128];
			for(int j = 0; j < 128; j++){
				desc[j] = (float)desc1.at<float>(i,j);
			}
			descriptor = new SurfFeatureDescriptor128(desc);
		}else{
			float * desc = new float[64];
			for(int j = 0; j < 64; j++){
				desc[j] = (float)desc1.at<float>(i,j);
			}
			descriptor = new SurfFeatureDescriptor64(desc);
		}

		kp->descriptor = descriptor;
		kp->r = r;
		kp->g = g;
		kp->b = b;
		if(z > 0 && !isnan(z)){
			kp->valid = true;
			kp->index_number = keypoints->valid_key_points.size();
			keypoints->valid_key_points.push_back(kp);
			kp->point = new Point(x,y,z,curr.pt.x,curr.pt.y);
		}else{
			kp->valid = false;
			kp->index_number = keypoints->invalid_key_points.size();
			keypoints->invalid_key_points.push_back(kp);
			kp->point = new Point(-x,-y,-z,curr.pt.x,curr.pt.y);
		}
	}

	sort(keypoints->valid_key_points.begin(),keypoints->valid_key_points.end(),comparison_Surf);
	sort(keypoints->invalid_key_points.begin(),keypoints->invalid_key_points.end(),comparison_Surf);

	gettimeofday(&end, NULL);
	double time = (end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec))/1000000.0f;
	if(verbose){printf("Extracted %i surf features in %fs\n",(int)kp1.size(),time);}
	//bool debugg = false;
	if(debugg){
		int width = rgb_img->width;
		int height = rgb_img->height;
		
		IplImage* img_combine = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);
		cvCopy( rgb_img, img_combine, NULL );

		for(unsigned int i = 0; i < keypoints->valid_key_points.size(); i++){
			KeyPoint * kp = keypoints->valid_key_points.at(i);
			cvCircle(img_combine,cvPoint(kp->w, kp->h), 3,cvScalar(0, 255, 0, 0),1, 8, 0);
		}

		for(unsigned int i = 0; i < keypoints->invalid_key_points.size(); i++){
			KeyPoint * kp = keypoints->invalid_key_points.at(i);
			cvCircle(img_combine,cvPoint(kp->w, kp->h), 3,cvScalar(0, 0, 255, 0),1, 8, 0);
		}


		cvNamedWindow("feature image", CV_WINDOW_AUTOSIZE );
		cvShowImage("feature image", img_combine);
		cvWaitKey(0);
		cvReleaseImage(&img_combine);
	}
    */
	return keypoints;

}
