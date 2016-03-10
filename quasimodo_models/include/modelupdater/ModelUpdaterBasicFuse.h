#ifndef reglibModelUpdaterBasicFuse_H
#define reglibModelUpdaterBasicFuse_H

#include "ModelUpdater.h"

#include "registration/nanoflann.hpp"

namespace reglib
{
    class ModelUpdaterBasicFuse: public ModelUpdater{
		public:

		std::vector<superpoint> fusedmodel;
		Registration * registration;

		//ModelGraph * graph;

        ModelUpdaterBasicFuse(Registration * registration_ = new Registration());
        ModelUpdaterBasicFuse(Model * model_,Registration * registration_ = new Registration());
        ~ModelUpdaterBasicFuse();

		virtual FusionResults registerModel(Model * model2, Eigen::Matrix4d guess = Eigen::Matrix4d::Identity(), double uncertanity = -1);
		//virtual void addFrame(RGBDFrame * frame, Eigen::Matrix4d pose);
		virtual void setRegistration( Registration * registration_);
		virtual void fuse(Model * model2, Eigen::Matrix4d guess = Eigen::Matrix4d::Identity(), double uncertanity = -1);
		virtual UpdatedModels fuseData(FusionResults * f, Model * model1, Model * model2);
		//virtual void refine();
		virtual	void computeMassRegistration(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames,std::vector<cv::Mat> current_masks);
	};

}

#endif // reglibModelUpdaterBasicFuse_H
