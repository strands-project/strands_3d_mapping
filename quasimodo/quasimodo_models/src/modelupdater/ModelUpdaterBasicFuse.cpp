#include "modelupdater/ModelUpdaterBasicFuse.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sys/time.h>
#include <emmintrin.h>

#include "registration/MassRegistration.h"

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
        printf("registerModel(%i %i)\n",int(model->id),int(model2->id));
		registration->viewer	= viewer;
		int step1 = std::max(int(model->frames.size())/1,1);
		int step2 = std::max(int(model2->frames.size())/1,1);//int step2 = 5;//std::min(int(model2->frames.size()),5);

		CloudData * cd1 = model ->getCD(model->points.size()/step1);
		registration->setDst(cd1);

		CloudData * cd2	= model2->getCD(model2->points.size()/step2);
		registration->setSrc(cd2);
		FusionResults fr = registration->getTransform(guess);

		printf("%i registerModel(%i %i)\n",__LINE__,int(model->id),int(model2->id));
        double best = -99999999999999;
        int best_id = -1;
		for(unsigned int ca = 0; ca < fr.candidates.size() && ca < 150; ca++){
            Eigen::Matrix4d pose = fr.candidates[ca];
            std::vector<Eigen::Matrix4d>	current_poses;
            std::vector<RGBDFrame*>			current_frames;
			//std::vector<cv::Mat>			current_masks;
			std::vector<ModelMask * >		current_modelmasks;

            for(unsigned int i = 0; i < model->frames.size(); i++){
				current_poses.push_back(				model->relativeposes[i]);
				current_frames.push_back(				model->frames[i]);
				//current_masks.push_back(				model->masks[i]);
				current_modelmasks.push_back(			model->modelmasks[i]);
            }

            for(unsigned int i = 0; i < model2->frames.size(); i++){
                current_poses.push_back(	pose	*	model2->relativeposes[i]);
                current_frames.push_back(				model2->frames[i]);
				//current_masks.push_back(				model2->masks[i]);
				current_modelmasks.push_back(			model2->modelmasks[i]);
            }

			//std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(current_poses, current_frames,current_masks,current_modelmasks);
			std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(current_poses, current_frames,current_modelmasks,false,10.0);
            std::vector<std::vector < float > > scores = getScores(ocs);
            std::vector<int> partition = getPartition(scores,2,5,2);

            double sumscore1 = 0;
            for(unsigned int i = 0; i < scores.size(); i++){
                for(unsigned int j = 0; j < scores.size(); j++){
                    if(i < model->frames.size() && j < model->frames.size()){sumscore1 += scores[i][j];}
                    if(i >= model->frames.size() && j >= model->frames.size()){sumscore1 += scores[i][j];}
                }
            }

            double sumscore2 = 0;
            for(unsigned int i = 0; i < scores.size(); i++){
                for(unsigned int j = 0; j < scores.size(); j++){
                    if(partition[i] == partition[j]){sumscore2 += scores[i][j];}
                }
            }

            double improvement = sumscore2-sumscore1;

            if(improvement > best){
                best = improvement;
                best_id = ca;
            }

			printf("tested %i with score: %f\n",ca,sumscore2);

            //printf("sumscore before: %f\n",sumscore1);
            //printf("sumscore after: %f\n",sumscore2);
            //printf("%i improvement: %f\n",ca,improvement);

            if(improvement > 0){
				//printf("%i improvement: %f\n",ca,improvement);
				//getOcclusionScores(current_poses, current_frames,current_masks,true);
            }
        }
        if(best_id != -1){
            fr.score = 9999999;
            fr.guess = fr.candidates[best_id];
        }

		
		//printf("score: %6.6f\n",fr.score);
		delete cd1;
		delete cd2;

		return fr;
	}
	return FusionResults();
}

void ModelUpdaterBasicFuse::fuse(Model * model2, Eigen::Matrix4d guess, double uncertanity){}

UpdatedModels ModelUpdaterBasicFuse::fuseData(FusionResults * f, Model * model1, Model * model2){
	printf("MODEL1 ");
	model1->print();

	printf("MODEL2 ");
	model2->print();

	UpdatedModels retval = UpdatedModels();
    Eigen::Matrix4d pose = f->guess;

	std::vector<Eigen::Matrix4d>	current_poses;
    std::vector<RGBDFrame*>			current_frames;
	std::vector<ModelMask*>			current_modelmasks;

    for(unsigned int i = 0; i < model1->frames.size(); i++){
		current_poses.push_back(				model1->relativeposes[i]);
		current_frames.push_back(				model1->frames[i]);
		current_modelmasks.push_back(			model1->modelmasks[i]);
	}

    for(unsigned int i = 0; i < model2->frames.size(); i++){
		current_poses.push_back(	pose	*	model2->relativeposes[i]);
        current_frames.push_back(				model2->frames[i]);
		current_modelmasks.push_back(			model2->modelmasks[i]);
	}

	double beforescore = model1->total_scores+model2->total_scores;
//	printf("beforescore: %f\n",beforescore);
	//std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(current_poses, current_frames,current_masks,current_modelmasks,false);
	std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(current_poses, current_frames,current_modelmasks,false);
	//std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(current_poses, current_frames,current_modelmasks,true);
    std::vector<std::vector < float > > scores = getScores(ocs);
	std::vector<int> partition = getPartition(scores,2,5,2);

//	printf("model1\n");
	unsigned int frames1 = model1->scores.size();
	unsigned int frames2 = model2->scores.size();
	double sumscore1 = 0;
	for(unsigned int i = 0; i < frames1; i++){
		for(unsigned int j = 0; j < frames1; j++){
			sumscore1 += scores[i][j];
		}
	}


//	printf("model2\n");
	double sumscore2 = 0;
	for(unsigned int i = 0; i < frames2; i++){
		for(unsigned int j = 0; j < frames2; j++){
			sumscore2 += scores[i+frames1][j+frames1];
		}
	}

	double sumscore = 0;
    for(unsigned int i = 0; i < scores.size(); i++){
        for(unsigned int j = 0; j < scores.size(); j++){
			if(partition[i] == partition[j]){sumscore += scores[i][j];}
		}
	}

	double improvement = sumscore-sumscore1-sumscore2;

	printf("sumscore before part: %f\n",sumscore1+sumscore2);
	printf("sumscore after  part: %f\n",sumscore);
	printf("improvement:          %f\n",improvement);

	std::vector<int> count;
	for(unsigned int i = 0; i < partition.size(); i++){
        if(int(count.size()) <= partition[i]){count.resize(partition[i]+1);}
		count[partition[i]]++;
	}

	int minpart = count[0];
	for(unsigned int i = 1; i < count.size(); i++){minpart = std::min(minpart,count[i]);}

    printf("partition: ");for(unsigned int i = 0; i < partition.size(); i++){printf("%i ",int(partition[i]));}printf("\n");

	if(count.size() == 1){
		model1->merge(model2,pose);
		model1->recomputeModelPoints();
		model1->scores = scores;
		model1->total_scores = sumscore;
		retval.updated_models.push_back(model1);
		retval.deleted_models.push_back(model2);
	}
	else if(improvement > 1){//Cannot fully fuse... separating...
//		printf("Cannot fully fuse...\n");
//exit(0);
//		retval.unchanged_models.push_back(model1);
//		retval.unchanged_models.push_back(model2);
//		return retval;

		int c = 0;

		int model1_ind = partition.front();
		bool model1_same = true;
        for(unsigned int i = 0; i < model1->frames.size(); i++){
			if(partition[c] != model1_ind){model1_same = false;}
			c++;
		}

		int model2_ind = partition.back();
		bool model2_same = true;
        for(unsigned int i = 0; i < model2->frames.size(); i++){
			if(partition[c] != model2_ind){model2_same = false;}
			c++;
		}

		if(!model1_same || !model2_same){//If something changed, update models
//			printf("SOMETHING CHANGED\n");
//exit(0);
            for(unsigned int i = 0; i < count.size(); i++){retval.new_models.push_back(new Model());}
			for(unsigned int i = 0; i < partition.size(); i++){
				retval.new_models[partition[i]]->frames.push_back(current_frames[i]);
				retval.new_models[partition[i]]->relativeposes.push_back(current_poses[i]);
				retval.new_models[partition[i]]->modelmasks.push_back(current_modelmasks[i]);
				//retval.new_models[partition[i]]->masks.push_back(current_masks[i].clone());

//				cv::namedWindow("mask",	cv::WINDOW_AUTOSIZE);
//				cv::imshow(		"mask",	retval.new_models[partition[i]]->masks.back());
//				cv::waitKey(0);
			}

            for(unsigned int part = 0; part < retval.new_models.size(); part++){
				Model * m = retval.new_models[part];
				m->recomputeModelPoints();
				m->scores.resize(m->frames.size());
                for(unsigned int i = 0; i < m->scores.size(); i++){
					m->scores[i].resize(m->scores.size());
                    for(unsigned int j = 0; j < m->scores.size(); j++){
						m->scores[i][j] = 0;
					}
				}

				std::vector<int> inds;
				for(unsigned int i = 0; i < partition.size(); i++){
                    if(partition[i] == int(part)){inds.push_back(i);}
				}

                for(unsigned int i = 0; i < m->scores.size(); i++){
                    for(unsigned int j = 0; j < m->scores.size(); j++){
						 m->scores[i][j] += scores[inds[i]][inds[j]];
					}
				}

				m->total_scores = 0;
                for(unsigned int i = 0; i < m->scores.size(); i++){
                    for(unsigned int j = 0; j < m->scores.size(); j++){
						m->total_scores += m->scores[i][j];
					}
				}

//				printf("total_scores %f\n",m->total_scores);
//				for(int i = 0; i < m->scores.size(); i++){
//					for(int j = 0; j < m->scores.size(); j++){
//						printf("%3.3f ",m->scores[i][j]*0.0001);
//					}
//					printf("\n");
//				}

				m->print();
			}

			retval.deleted_models.push_back(model1);
			retval.deleted_models.push_back(model2);
		}else{//
			retval.unchanged_models.push_back(model1);
			retval.unchanged_models.push_back(model2);
		}
		return retval;
//		MassRegistrationPPR * massreg = new MassRegistrationPPR(0.05,false);
//		massreg->setData(current_frames,current_masks);
//		MassFusionResults mfr = massreg->getTransforms(current_poses);
//		current_poses = mfr.poses;
//		std::vector<std::vector < OcclusionScore > > ocs2 = getOcclusionScores(current_poses, current_frames,current_masks,current_modelmasks);
//        std::vector<std::vector < float > > scores2 = getScores(ocs2);
//		std::vector<int> partition2 = getPartition(scores2,2,5,2);

//		double sumscore_after = 0;
//		for(int i = 0; i < scores2.size(); i++){
//			for(int j = 0; j < scores2.size(); j++){
//				if(partition2[i] == partition2[j]){sumscore_after += scores2[i][j];}
//			}
//		}

//		if(sumscore_after > sumscore){
//			printf("sumscore after: %f\n",sumscore_after);

//			std::vector<int> count2;
//			for(unsigned int i = 0; i < partition2.size(); i++){
//				if(count2.size() <= partition2[i]){count2.resize(partition2[i]+1);}
//				count2[partition2[i]]++;
//			}

//			int minpart2 = count2[0];
//			for(unsigned int i = 1; i < count2.size(); i++){minpart2 = std::min(minpart2,count2[i]);}

//			printf("----------------------------\n");
//			printf("partition2: ");for(unsigned int i = 0; i < partition2.size(); i++){printf("%i ",partition2[i]);}printf("\n");
//			for(unsigned int i = 0; i < count2.size(); i++){printf("count2 %i -> %i\n",i,count2[i]);}

//			if(count2.size() == 1){//Mass registration solved the problem, GREAT
//				printf("Mass registration solved the problem, GREAT\n");
//				for(unsigned int i = 0; i < model2->frames.size();i++){
//					model1->frames.push_back(model2->frames[i]);
//					model1->masks.push_back(model2->masks[i]);
//				}
//				model1->relativeposes = current_poses;
//				model1->recomputeModelPoints();

//				retval.updated_models.push_back(model1);
//				retval.deleted_models.push_back(model2);
//			}else{//Mass registration did NOT solve the problem
//				printf("Mass registration did NOT solve the problem UPDATE THIS PART\n");

//				//Check if identical to input partition
//				int c = 0;

//				int model1_ind = partition2.front();
//				bool model1_same = true;
//				for(int i = 0; i < model1->frames.size(); i++){
//					if(partition2[c] != model1_ind){model1_same = false;}
//					c++;
//				}

//				int model2_ind = partition2.back();
//				bool model2_same = true;
//				for(int i = 0; i < model2->frames.size(); i++){
//					if(partition2[c] != model2_ind){model2_same = false;}
//					c++;
//				}

//				if(!model1_same || !model2_same){//If something changed, update models
//					printf("SOMETHING CHANGED\n");
//					retval.updated_models.push_back(model1);
//					retval.updated_models.push_back(model2);
//					exit(0);
//				}else{//
//					retval.unchanged_models.push_back(model1);
//					retval.unchanged_models.push_back(model2);
//				}

//			}
//		}
	}

	retval.unchanged_models.push_back(model1);
	retval.unchanged_models.push_back(model2);
	return retval;
}

void ModelUpdaterBasicFuse::computeMassRegistration(std::vector<Eigen::Matrix4d> current_poses, std::vector<RGBDFrame*> current_frames,std::vector<cv::Mat> current_masks){
	printf("void ModelUpdaterBasicFuse::computeMassRegistration\n");
/*
	MassRegistrationPPR * massreg = new MassRegistrationPPR();
	//massreg->viewer = viewer;
	massreg->setData(current_frames,current_masks);
	MassFusionResults mfr = massreg->getTransforms(current_poses);

	std::vector<std::vector < OcclusionScore > > ocs = getOcclusionScores(current_poses, current_frames,current_masks);
	std::vector<std::vector < float > > scores = getScores(ocs,10);
	std::vector<int> partition = getPartition(scores,2,5,2);
	printf("new partition: ");for(unsigned int i = 0; i < partition.size(); i++){printf("%i ",partition[i]);}printf("\n");
*/
	printf("WARNING: THIS METHOD NOT IMPLEMENTED\n");
	exit(0);
}

//void ModelUpdaterBasicFuse::refine(){}//No refinement behaviour added yet

void ModelUpdaterBasicFuse::setRegistration( Registration * registration_){
	if(registration != 0){delete registration;}
	registration = registration;
}

}


