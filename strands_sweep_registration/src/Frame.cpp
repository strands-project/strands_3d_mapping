#include "strands_sweep_registration/Frame.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <fstream>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>

int frame_id = 0;

Frame::Frame(Camera * camera_, float * rgb_data_, float * depth_data_){
	id = frame_id;
	frame_id++;
	camera = camera_;

	cv::Mat img = cv::Mat::zeros(camera->height,camera->width, CV_8UC1);
	unsigned char * imgdata = (unsigned char*)img.data;
	for(int w = 0; w < camera->width; w++){
		for(int h = 0; h < camera->height; h++){
			int i = h*camera->width+w;
			int i3		= 3*i;
			unsigned char r		= rgb_data_[i3+0];
			unsigned char g		= rgb_data_[i3+1];
			unsigned char b		= rgb_data_[i3+2];
			imgdata[i] = (unsigned char)(r);
		}
	}

	cv::ORB orb = cv::ORB(600,2.5f, 1, 3, 0,2, cv::ORB::HARRIS_SCORE, 31);	
	orb(img, cv::Mat(), keypoints, descriptors);
	featuretype = 0;

	float centerX		= camera->cx;
	float centerY		= camera->cy;
	float invFocalX		= 1.0f/camera->fx;
    float invFocalY		= 1.0f/camera->fy;

	for( int i = 0; i < (int)keypoints.size(); i++ ){
		float w = keypoints.at(i).pt.x;
		float h = keypoints.at(i).pt.y;
		
		int id = int(h+0.5)*camera->width+int(w+0.5);
		float z = depth_data_[id];
		if( z == 0 || z > 4.5 || isnan(z)){
			keypoints.at(i) = keypoints.back();
			keypoints.pop_back();
			i--;
		}else{
			keypoint_depth.push_back(z);
			float x = (w - centerX) * z * invFocalX;
		   	float y = (h - centerY) * z * invFocalY;
			keypoint_location.push_back(Eigen::Vector3f(x,y,z));
		}
	}

//    cv::drawKeypoints(img, keypoints, img);
//    cv::imshow( "Display window", img);                   // Show our image inside it.
//    cv::waitKey(0);                                          // Wait for a keystroke in t

	recalculateFullPoints();
}

Frame::Frame(Camera * camera_, std::vector<cv::KeyPoint> k, std::vector<double> depth, cv::Mat d)
{
    id = frame_id;
    frame_id++;
    camera = camera_;
    featuretype = 0;

    keypoints = k;
    descriptors = d;

    float centerX		= camera->cx;
    float centerY		= camera->cy;
    float invFocalX		= 1.0f/camera->fx;
    float invFocalY		= 1.0f/camera->fy;

    std::vector<int> good_indices;
    for (size_t i=0; i<depth.size();i++)
    {
        float z = (float)depth[i];
        if( !(z == 0 || z > 4.5 || isnan(z))){
            good_indices.push_back(i);
        }
    }

    std::vector<cv::KeyPoint> good_keypoints;
    std::vector<double> good_depth;
    cv::Mat good_descriptors;
    for (auto index : good_indices)
    {
        good_keypoints.push_back(k[index]);
        good_depth.push_back(depth[index]);
        good_descriptors.push_back(d.row(index));
    }
    depth = good_depth;
    keypoints = good_keypoints;
    descriptors = good_descriptors;

    for( int i = 0; i < (int)keypoints.size(); i++ ){
        double w = keypoints.at(i).pt.x;
        double h = keypoints.at(i).pt.y;

        float z = (float)depth[i];
        if( z == 0 || z > 4.5 || isnan(z)){
            keypoints.at(i) = keypoints.back();
            keypoints.pop_back();

            depth.at(i) = depth.back();
            depth.pop_back();
            std::cout<<"------------------ RESHUFFLING -------------------"<<std::endl;
            i--;
        }else{
            keypoint_depth.push_back(z);
            float x = (w - centerX) * z * invFocalX;
            float y = (h - centerY) * z * invFocalY;
            keypoint_location.push_back(Eigen::Vector3f(x,y,z));
        }
    }

    recalculateFullPoints();
}

void Frame::recalculateFullPoints(){
	float centerX		= camera->cx;
	float centerY		= camera->cy;
	float invFocalX		= 1.0f/camera->fx;
    float invFocalY		= 1.0f/camera->fy;
	for(unsigned int i = 0; i < keypoints.size(); i++){
		float w = keypoints.at(i).pt.x;
		float h = keypoints.at(i).pt.y;
		float z = keypoint_depth.at(i);
		float x = (w - centerX) * z * invFocalX;
	   	float y = (h - centerY) * z * invFocalY;

		keypoint_location.push_back(Eigen::Vector3f(x,y,z));
	}
}

Frame::~Frame(){};
