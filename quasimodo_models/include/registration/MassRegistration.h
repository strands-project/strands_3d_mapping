#ifndef MassRegistration_H
#define MassRegistration_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>

#include <Eigen/Dense>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "ICP.h"

#include "../weightfunctions/DistanceWeightFunction2.h"
#include "RegistrationSICP.h"
#include "../core/RGBDFrame.h"

namespace reglib
{
/*
	enum MatchType { PointToPoint, PointToPlane };

	class CloudData{
		public:
		Eigen::MatrixXd information;
		Eigen::MatrixXd data;
		Eigen::MatrixXd normals;
	
		CloudData();
		~CloudData();
	};
*/
	class MassFusionResults{
		public:

		std::vector<Eigen::Matrix4d> poses;
		double score;

		MassFusionResults(){score = -1;}
		MassFusionResults(std::vector<Eigen::Matrix4d> poses_, double score_){
			poses = poses_;
			score = score_;
		}
		~MassFusionResults(){}
	};

	class MassRegistration{
		public:
		
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

		unsigned int visualizationLvl;

		std::vector<RGBDFrame*> frames;
		std::vector<cv::Mat> masks;

		MassRegistration();
		~MassRegistration();

		void setData(std::vector<RGBDFrame*> frames_,std::vector<cv::Mat> masks_);
		void setVisualizationLvl(unsigned int lvl);
		virtual MassFusionResults getTransforms(std::vector<Eigen::Matrix4d> guess);

		virtual void show(Eigen::MatrixXd X, Eigen::MatrixXd Y);

		virtual void show(std::vector<Eigen::MatrixXd> Xv, bool save = false, std::string filename = "", bool stop = true);
		//std::vector<Eigen::Matrix4d> current_poses,
/*
		CloudData * src;
		CloudData * dst;




		
		void setSrc(CloudData * src_);
		void setDst(CloudData * dst_);



		virtual void show(Eigen::MatrixXd X, Eigen::MatrixXd Y);
		virtual void show(Eigen::MatrixXd X, Eigen::MatrixXd Y, Eigen::VectorXd W);
		virtual void show(Eigen::MatrixXd X, Eigen::MatrixXd Xn, Eigen::MatrixXd Y, Eigen::MatrixXd Yn);
*/
	};

}

//#include "RegistrationSICP.h"
#include "MassRegistrationPPR.h"
//#include "RegistrationGOICP.h"
#endif // MassRegistration_H
