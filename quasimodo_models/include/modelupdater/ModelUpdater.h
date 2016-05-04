#ifndef reglibModelUpdater_H
#define reglibModelUpdater_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h> 
#include <chrono>

#include <Eigen/Dense>
#include "../model/Model.h"
#include "../registration/Registration.h"
#include "../registration/MassRegistration.h"
#include "../mesher/Mesh.h"


#include <pcl/visualization/pcl_visualizer.h>

namespace reglib
{
	class UpdatedModels{
		public:
		std::vector< Model * > new_models;
		std::vector< Model * > updated_models;
		std::vector< Model * > unchanged_models;
		std::vector< Model * > deleted_models;
		
		UpdatedModels(){}
	};
	
	class OcclusionScore{
		public:
		double score;
		double occlusions;

		OcclusionScore(){score = 0;occlusions = 0;}
		OcclusionScore(	double score_ ,double occlusions_){score = score_;occlusions = occlusions_;}
		~OcclusionScore(){}
	};

	class ModelUpdater{
        public:
        double occlusion_penalty;
        double massreg_timeout;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
		Model * model;
		Mesh * mesher;

		ModelUpdater();
		ModelUpdater(Model * model_);
		~ModelUpdater();


		virtual FusionResults registerModel(Model * model2, Eigen::Matrix4d guess = Eigen::Matrix4d::Identity(), double uncertanity = -1);
		virtual void fuse(Model * model2, Eigen::Matrix4d guess = Eigen::Matrix4d::Identity(), double uncertanity = -1);
		virtual UpdatedModels fuseData(FusionResults * f, Model * model1,Model * model2);
		virtual void refine(double reg = 0.05,bool useFullMask = false);
		virtual void show(bool stop = true);
		virtual void pruneOcclusions();
//		virtual OcclusionScore computeOcclusionScore(RGBDFrame * src, cv::Mat src_mask, ModelMask * src_modelmask, RGBDFrame * dst, cv::Mat dst_mask, ModelMask * dst_modelmask, Eigen::Matrix4d p, int step = 1, bool debugg = false);
		virtual OcclusionScore computeOcclusionScore(RGBDFrame * src, ModelMask * src_modelmask, RGBDFrame * dst, ModelMask * dst_modelmask, Eigen::Matrix4d p, int step = 1, bool debugg = false);
		virtual void computeResiduals(std::vector<float> & residuals, std::vector<float> & weights, RGBDFrame * src, cv::Mat src_mask, ModelMask * src_modelmask, RGBDFrame * dst, cv::Mat dst_mask, ModelMask * dst_modelmask, Eigen::Matrix4d p, bool debugg = false);

		virtual std::vector<std::vector< OcclusionScore > >	computeAllOcclusionScores(	RGBDFrame * src, cv::Mat src_mask, RGBDFrame * dst, cv::Mat dst_mask,Eigen::Matrix4d p, bool debugg = false);
		virtual	void computeMassRegistration(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames,std::vector<cv::Mat> current_masks);

		std::vector<std::vector < OcclusionScore > > getOcclusionScores(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames, std::vector<ModelMask*> current_modelmasks,  bool debugg_scores = false, double speedup = 1.0);
        std::vector<std::vector < float > > getScores(std::vector<std::vector < OcclusionScore > > occlusionScores);
		std::vector<int> getPartition(std::vector< std::vector< float > > & scores, int dims = 2, int nr_todo = 5, double timelimit = 2);

		virtual void recomputeScores();

		CloudData * getCD(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames,std::vector<cv::Mat> current_masks, int step);
	};

}

#include "ModelUpdaterBasic.h"
#include "ModelUpdaterBasicFuse.h"
#endif // reglibModelUpdater_H
