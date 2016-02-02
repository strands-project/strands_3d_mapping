#ifndef reglibModelUpdaterBasic_H
#define reglibModelUpdaterBasic_H

#include "ModelUpdater.h"

namespace reglib
{
	class ModelUpdaterBasic: public ModelUpdater{
		public:

		Registration * registration;

		ModelUpdaterBasic(Registration * registration_ = new Registration());
		ModelUpdaterBasic(Model * model_,Registration * registration_ = new Registration());
		~ModelUpdaterBasic();

		virtual FusionResults registerModel(Model * model2, Eigen::Matrix4d guess = Eigen::Matrix4d::Identity(), double uncertanity = -1);

		virtual void setRegistration( Registration * registration_);
		virtual void fuse(Model * model2, Eigen::Matrix4d guess = Eigen::Matrix4d::Identity(), double uncertanity = -1);
		virtual void refine();
	};

}

#endif // reglibModelUpdaterBasic_H
