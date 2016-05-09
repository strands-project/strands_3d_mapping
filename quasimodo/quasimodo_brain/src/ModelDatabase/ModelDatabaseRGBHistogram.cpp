#include "ModelDatabaseRGBHistogram.h"

ModelDatabaseRGBHistogram::ModelDatabaseRGBHistogram(int res_){
	res = res_;
	printf("made a ModelDatabaseRGBHistogram(%i)\n",res);
}
ModelDatabaseRGBHistogram::~ModelDatabaseRGBHistogram(){}

std::vector< double > getDescriptor(int res, reglib::Model * model){
	std::vector< double > descriptor;
	descriptor.resize(res*res*res);
	for(int i = 0; i < res*res*res; i++){descriptor[i] = 0;}
	std::vector<reglib::superpoint> & points = model->points;
	for(unsigned int i = 0; i < points.size(); i++){
		reglib::superpoint & sp = points[i];
		int r = sp.feature(0);
		int g = sp.feature(1);
		int b = sp.feature(2);
		int rind = int(double(res)*r/256.0);
		int gind = int(double(res)*g/256.0);
		int bind = int(double(res)*b/256.0);
		int ind = res*res*rind + res*gind + bind;
		descriptor[ind]++;
	}
	for(int i = 0; i < res*res*res; i++){descriptor[i] /= double(points.size());}
	return descriptor;
}

void ModelDatabaseRGBHistogram::add(reglib::Model * model){
	//std::vector< std::vector< double > > descriptors;
	std::vector< double > descriptor = getDescriptor(res,model);
	descriptors.push_back(descriptor);
	models.push_back(model);
	//printf("number of models: %i\n",models.size());
}

bool ModelDatabaseRGBHistogram::remove(reglib::Model * model){
	for(unsigned int i = 0; i < models.size(); i++){
		if(models[i] == model){
			models[i] = models.back();
			models.pop_back();
			descriptors[i] = descriptors.back();
			descriptors.pop_back();
			return true;
		}	
	}
	return false;
}

double dist(std::vector< double > descriptorA, std::vector< double > descriptorB){
	double sum = 0;
	for(unsigned int i = 0; i < descriptorA.size(); i++){
		sum += std::min(descriptorA[i],descriptorB[i]);
	}
	return sum;
}

bool ModelDatabaseRGBHistogram_sort (std::pair<double,reglib::Model *> i, std::pair<double,reglib::Model *> j) { return (i.first>j.first); }

std::vector<reglib::Model *> ModelDatabaseRGBHistogram::search(reglib::Model * model, int number_of_matches){
	std::vector< double > descriptor = getDescriptor(res,model);

	std::vector< std::pair<double,reglib::Model *> > distances;

	std::vector<reglib::Model *> ret;
	printf("when searching my database contains %i models\n",models.size());
	for(unsigned int i = 0; i < models.size(); i++){
		if(models[i] != model){
			distances.push_back(std::make_pair(dist(descriptor,descriptors[i]),models[i]));
		}
		//if(ret.size() == number_of_matches){break;}
	}

	std::sort (distances.begin(), distances.end(), ModelDatabaseRGBHistogram_sort);

	for(unsigned int i = 0; i < distances.size() && i < number_of_matches; i++){
		printf("%i -> %f\n",i,distances[i].first);
		ret.push_back(distances[i].second);
	}

	return ret;
}
