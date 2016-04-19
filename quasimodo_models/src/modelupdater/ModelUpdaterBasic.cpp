#include "modelupdater/ModelUpdaterBasic.h"

namespace reglib
{

ModelUpdaterBasic::ModelUpdaterBasic(Registration * registration_){
	registration = registration_;
	model = new reglib::Model();
}

ModelUpdaterBasic::ModelUpdaterBasic(Model * model_, Registration * registration_){
    registration = registration_;
    model = model_;
}

ModelUpdaterBasic::~ModelUpdaterBasic(){}

FusionResults ModelUpdaterBasic::registerModel(Model * model2, Eigen::Matrix4d guess, double uncertanity){
	printf("registerModel\n");
	return FusionResults();
}

void ModelUpdaterBasic::fuse(Model * model2, Eigen::Matrix4d guess, double uncertanity){}
void ModelUpdaterBasic::refine(){}//No refinement behaviour added yet

void ModelUpdaterBasic::setRegistration( Registration * registration_){
	if(registration != 0){delete registration;}
	registration = registration;
}

}


