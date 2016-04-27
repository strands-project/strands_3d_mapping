#ifndef reglibModel_H
#define reglibModel_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>

#include <Eigen/Dense>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../core/RGBDFrame.h"
#include "../registration/Registration.h"

#include "ModelMask.h"

namespace reglib
{

class superpoint{
	public:
	Eigen::Vector3f point;
	Eigen::Vector3f normal;
	Eigen::VectorXf feature;
	double point_information;
	double feature_information;
	int last_update_frame_id;

	superpoint(Eigen::Vector3f p, Eigen::Vector3f n, Eigen::VectorXf f, double pi = 1, double fi = 1, int id = 0){
		point = p;
		normal = n;
		feature = f;
		point_information = pi;
		feature_information = fi;
		last_update_frame_id = id;
	}

	~superpoint(){}

	void merge(superpoint p, double weight = 1){
		point = weight*p.point_information*p.point + point_information*point;
		point /= weight*p.point_information + point_information;
		point_information = weight*p.point_information + point_information;

		normal = weight*p.point_information*p.normal + point_information*normal;
		normal.normalize();

		feature = weight*p.feature_information*p.feature + feature_information*feature;
		feature /= weight*p.feature_information + feature_information;
		feature_information = weight*p.feature_information + feature_information;
		last_update_frame_id = std::max(p.last_update_frame_id,last_update_frame_id);
	}
};

	class Model{
		public:

        double score;
		unsigned long id;

		int last_changed;

		std::vector<superpoint> points;

		std::vector<Eigen::Matrix4d> relativeposes;
		std::vector<RGBDFrame*> frames;
		//std::vector<cv::Mat> masks;
		std::vector<ModelMask*> modelmasks;

		double total_scores;
		std::vector<std::vector < float > > scores;

		Model();
		Model(RGBDFrame * frame_, cv::Mat mask, Eigen::Matrix4d pose = Eigen::Matrix4d::Identity());
		~Model();
		
		void merge(Model * model, Eigen::Matrix4d p);

		void recomputeModelPoints();
		void addPointsToModel(RGBDFrame * frame, ModelMask * modelmask, Eigen::Matrix4d p);

		//void addFrameToModel(RGBDFrame * frame, cv::Mat mask, Eigen::Matrix4d p);
		void addFrameToModel(RGBDFrame * frame, ModelMask * modelmask, Eigen::Matrix4d p);
		CloudData * getCD(unsigned int target_points = 2000);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPCLcloud(int step = 5, bool color = true);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr getPCLnormalcloud(int step = 5, bool color = true);
		void print();
		void save(std::string path = "");
		static Model * load(Camera * cam, std::string path);
		bool testFrame(int ind = 0);
	};

}

#endif // reglibModel_H
