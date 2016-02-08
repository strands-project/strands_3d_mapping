#include "ModelUpdaterBasicFuse.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sys/time.h>
#include <emmintrin.h>

#include "MassRegistration.h"

namespace reglib
{

//------------MODEL UPDATER-----------
ModelUpdaterBasicFuse::ModelUpdaterBasicFuse(Registration * registration_){
	registration = registration_;
	model = new reglib::Model();
}

ModelUpdaterBasicFuse::ModelUpdaterBasicFuse(Model * model_, Registration * registration_){
	registration = registration_;
	model = model_;
}

ModelUpdaterBasicFuse::~ModelUpdaterBasicFuse(){}

FusionResults ModelUpdaterBasicFuse::registerModel(Model * model2, Eigen::Matrix4d guess, double uncertanity){
	if(model->frames.size() > 0){
		registration->viewer	= viewer;
		int step1 = std::max(int(model->frames.size())/2,5);
		CloudData * cd1 = model ->getCD(model->points.size()/step1);
		registration->setDst(cd1);
		int step2 = 5;//std::min(int(model2->frames.size()),5);
		CloudData * cd2	= model2->getCD(model2->points.size()/step2);
		registration->setSrc(cd2);
		FusionResults fr = registration->getTransform(guess);
		//fr.score = -1;
		
		printf("score: %6.6f\n",fr.score);
/*
		if(fr.score > 2000){
			Eigen::Matrix4d pose = fr.guess;

			std::vector<Eigen::Matrix4d>	current_poses;
			std::vector<RGBDFrame*>			current_frames;
			std::vector<cv::Mat>			current_masks;

			for(int i = 0; i < model->frames.size(); i++){
				current_poses.push_back(	model->relativeposes[i]);
				current_frames.push_back(	model->frames[i]);
				current_masks.push_back(	model->masks[i]);
			}

			for(int i = 0; i < model2->frames.size(); i++){
				current_poses.push_back(	pose*model2->relativeposes[i]);
				current_frames.push_back(	model2->frames[i]);
				current_masks.push_back(	model2->masks[i]);
			}
			
			std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(current_poses, current_frames,current_masks);
			std::vector<std::vector < float > > scores = getScores(ocs,10);
			std::vector<int> partition = getPartition(scores,3,5,2);
			
			std::vector<int> count;
			for(unsigned int i = 0; i < partition.size(); i++){
				if(count.size() <= partition[i]){count.resize(partition[i]+1);}
				count[partition[i]]++;
			}

			int minpart = count[0];
			for(unsigned int i = 1; i < count.size(); i++){minpart = std::min(minpart,count[i]);}

			printf("----------------------------\n");
			printf("partition: ");for(unsigned int i = 0; i < partition.size(); i++){printf("%i ",partition[i]);}printf("\n");
			for(unsigned int i = 0; i < count.size(); i++){printf("count %i -> %i\n",i,count[i]);}

			exit(0);
		}else{fr.score = -1;}
*/
		delete cd1;
		delete cd2;
		return fr;
	}
	return FusionResults();
}

void ModelUpdaterBasicFuse::fuse(Model * model2, Eigen::Matrix4d guess, double uncertanity){
	printf("void ModelUpdaterBasicFuse::fuse\n");
	//Blind addidtion of new model to old model...
	if(model->frames.size() > 0){
/*
		registration->viewer	= viewer;
		CloudData * cd1;
		CloudData * cd2;

		cd1 = model ->getCD(model->points.size()/100);
		registration->setDst(cd1);
		cd2	= model2->getCD(1000);
		registration->setSrc(cd2);
		guess = registration->getTransform(guess);
		delete cd1;
		delete cd2;

		cd1 = model ->getCD(model->points.size());
		registration->setDst(cd1);
		cd2	= model2->getCD(10000);
		registration->setSrc(cd2);
		guess = registration->getTransform(guess);
		delete cd2;

		//cout << guess << endl;
		delete cd1;
*/
	}
//	FusionResults * f = new FusionResults(guess,-1);
//	fuseData(f,model,model2);
//	delete f;
}

UpdatedModels ModelUpdaterBasicFuse::fuseData(FusionResults * f, Model * model1, Model * model2){
	UpdatedModels retval = UpdatedModels();
	Eigen::Matrix4d pose;

	printf("void ModelUpdaterBasicFuse::fuseData\n");
/*
	registration->only_initial_guess = true;

	int step1 = std::max(int(model->frames.size())/2,5);
	CloudData * cd1 = model ->getCD(model->points.size()/step1);
	registration->setDst(cd1);
	int step2 = 5;
	CloudData * cd2	= model2->getCD(model2->points.size()/step2);
	registration->setSrc(cd2);

	Eigen::Matrix4d pose = f->guess;
	FusionResults fr = registration->getTransform(pose);

	registration->only_initial_guess = false;
*/
	pose = f->guess;

	std::vector<Eigen::Matrix4d>	current_poses;
	std::vector<RGBDFrame*>			current_frames;
	std::vector<cv::Mat>			current_masks;

	for(int i = 0; i < model1->frames.size(); i++){
		current_poses.push_back(	model1->relativeposes[i]);
		current_frames.push_back(	model1->frames[i]);
		current_masks.push_back(	model1->masks[i]);
	}

	for(int i = 0; i < model2->frames.size(); i++){
		current_poses.push_back(	pose	*	model2->relativeposes[i]);
		current_frames.push_back(				model2->frames[i]);
		current_masks.push_back(				model2->masks[i]);
	}

	std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(current_poses, current_frames,current_masks);
	std::vector<std::vector < float > > scores = getScores(ocs,10);
	std::vector<int> partition = getPartition(scores,2,5,2);

	//MassRegistrationPPR * massreg = new MassRegistrationPPR();
	//massreg->setData(current_frames,current_masks);
	//MassFusionResults mfr = massreg->getTransforms(current_poses);


	std::vector<int> count;
	for(unsigned int i = 0; i < partition.size(); i++){
		if(count.size() <= partition[i]){count.resize(partition[i]+1);}
		count[partition[i]]++;
	}

	int minpart = count[0];
	for(unsigned int i = 1; i < count.size(); i++){minpart = std::min(minpart,count[i]);}

	printf("----------------------------\n");
	printf("partition: ");for(unsigned int i = 0; i < partition.size(); i++){printf("%i ",partition[i]);}printf("\n");
	for(unsigned int i = 0; i < count.size(); i++){printf("count %i -> %i\n",i,count[i]);}

	if(count.size() == 1){
		
		for(unsigned int i = 0; i < model2->frames.size();i++){
			model1->addFrameToModel(model2->frames[i], model2->masks[i],pose*model2->relativeposes[i]);
		}
	
		//if(current_poses.size() % 5 == 0){
			printf("time to refine again\n");

			MassRegistrationPPR * massreg = new MassRegistrationPPR(0.05,false);
			massreg->setData(current_frames,current_masks);
			MassFusionResults mfr = massreg->getTransforms(current_poses);

			//maybe check to ensure still coherent
			model1->relativeposes = mfr.poses;
			model1->recomputeModelPoints();
		//}

		retval.updated_models.push_back(model1);
		retval.deleted_models.push_back(model2);
	}
	else{//Cannot fully fuse...
		printf("Cannot fully fuse...\n");
		//Atempt mass registration to solve problem
		
		MassRegistrationPPR * massreg = new MassRegistrationPPR(0.05,false);
		massreg->setData(current_frames,current_masks);
		MassFusionResults mfr = massreg->getTransforms(current_poses);
		current_poses = mfr.poses;
		
		std::vector<std::vector < OcclusionScore > > ocs2 = getOcclusionScores(current_poses, current_frames,current_masks);
		std::vector<std::vector < float > > scores2 = getScores(ocs2,10);
		std::vector<int> partition2 = getPartition(scores2,2,5,2);

		std::vector<int> count2;
		for(unsigned int i = 0; i < partition2.size(); i++){
			if(count2.size() <= partition2[i]){count2.resize(partition2[i]+1);}
			count2[partition2[i]]++;
		}

		int minpart2 = count2[0];
		for(unsigned int i = 1; i < count2.size(); i++){minpart2 = std::min(minpart2,count2[i]);}

		printf("----------------------------\n");
		printf("partition2: ");for(unsigned int i = 0; i < partition2.size(); i++){printf("%i ",partition2[i]);}printf("\n");
		for(unsigned int i = 0; i < count2.size(); i++){printf("count2 %i -> %i\n",i,count2[i]);}
	
		if(count2.size() == 1){//Mass registration solved the problem, GREAT
			printf("Mass registration solved the problem, GREAT\n");
			for(unsigned int i = 0; i < model2->frames.size();i++){
				model1->frames.push_back(model2->frames[i]);
				model1->masks.push_back(model2->masks[i]);
			}
			model1->relativeposes = current_poses;
			model1->recomputeModelPoints();
			
			retval.updated_models.push_back(model1);
			retval.deleted_models.push_back(model2);
		}else{//Mass registration did NOT solve the problem
			printf("Mass registration did NOT solve the problem UPDATE THIS PART\n");
			
			//if(current_frames.size() > 10){
			//	for(int i = 0; i < current_frames.size(); i++){
			//		scores[i][i] = 0;
			//		for(int j = i+1; j < current_frames.size(); j++){
			//			if(( ocs2[i][j].score+ocs2[j][i].score - 2*(ocs2[i][j].occlusions+ocs2[j][i].occlusions)) < 0){
			//				Eigen::Matrix4d relative_pose = current_poses[i].inverse() * current_poses[j];
			//				computeOcclusionScore(current_frames[j], current_masks[j], current_frames[i], current_masks[i],relative_pose,true);
			//				computeOcclusionScore(current_frames[i], current_masks[i], current_frames[j], current_masks[j],relative_pose.inverse(),true);
			//			}
			//		}
			//	}
			//}
			

			//Check if identical to input partition
			int c = 0;
			
			int model1_ind = partition2.front();
			bool model1_same = true;
			for(int i = 0; i < model1->frames.size(); i++){
				if(partition2[c] != model1_ind){model1_same = false;}
				c++;
			}

			int model2_ind = partition2.back();
			bool model2_same = true;
			for(int i = 0; i < model2->frames.size(); i++){
				if(partition2[c] != model2_ind){model2_same = false;}
				c++;
			}
			
			if(!model1_same || !model2_same){//If something changed, update models
				printf("SOMETHING CHANGED\n");
				exit(0);
			}else{//

			}
			//if not update

			retval.updated_models.push_back(model1);
			retval.updated_models.push_back(model2);
		}
	}

	return retval;
}

void ModelUpdaterBasicFuse::computeMassRegistration(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames,std::vector<cv::Mat> current_masks){
printf("void ModelUpdaterBasicFuse::computeMassRegistration\n");

MassRegistrationPPR * massreg = new MassRegistrationPPR();
//massreg->viewer = viewer;
massreg->setData(current_frames,current_masks);
MassFusionResults mfr = massreg->getTransforms(current_poses);

std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(current_poses, current_frames,current_masks);
std::vector<std::vector < float > > scores = getScores(ocs,10);
std::vector<int> partition = getPartition(scores,2,5,2);
printf("new partition: ");for(unsigned int i = 0; i < partition.size(); i++){printf("%i ",partition[i]);}printf("\n");

}

void ModelUpdaterBasicFuse::refine(){}//No refinement behaviour added yet

void ModelUpdaterBasicFuse::setRegistration( Registration * registration_){
	if(registration != 0){delete registration;}
	registration = registration;
}

}


