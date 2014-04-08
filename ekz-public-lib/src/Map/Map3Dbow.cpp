#include "Map3Dbow.h"

using namespace std;

Map3Dbow::Map3Dbow(string file_path){
	path=file_path;
	nr_restarts_ = 1;
	iterations_ = 30;
	nr_clusters_ = 500;
}

Map3Dbow::Map3Dbow(string file_path, int nr_restarts, int iterations, int nr_clusters){
	path=file_path;
	nr_restarts_ = nr_restarts;
	iterations_ = iterations;
	nr_clusters_ = nr_clusters;
}

Map3Dbow::~Map3Dbow(){}

vector<Matrix4f> Map3Dbow::estimate(){
	if(verbose){printf("estimate\n");}
	vector<FeatureDescriptor *> descriptors;
	for(unsigned int i  = 0; i < frames.size(); i++){
		KeyPointSet * current = frames.at(i)->keypoints;
		for(unsigned int j = 0; j < current->valid_key_points.size(); j++){descriptors.push_back(current->valid_key_points.at(j)->descriptor);}
		for(unsigned int j = 0; j < current->invalid_key_points.size(); j++){descriptors.push_back(current->invalid_key_points.at(j)->descriptor);}	
	}
	vector<FeatureDescriptor * > * bags = kmeans(descriptors, nr_restarts_, iterations_, nr_clusters_);
	
	for(unsigned int i = 0; i < bags->size(); i++){
		char buff[250];
		sprintf(buff,"%s_%i",path.c_str(),(int)i);
		bags->at(i)->store(string(buff));
		//bags->at(i)->print();
	}
	if(verbose){printf("estimate done\n");}
	return vector<Matrix4f>();
}

vector<FeatureDescriptor * > * Map3Dbow::kmeans(vector<FeatureDescriptor *> input, int nr_restarts, int iterations, int nr_clusters){
	if(verbose){printf("doing kmeans with %i features\n",(int)input.size());}
	float best_sum = -1;
	vector<FeatureDescriptor * > * best_centers = new vector<FeatureDescriptor * >();
	for(int restart = 0; restart < nr_restarts; restart++)
	{
		if(verbose){printf("--------------------------------------------------------------------------------\n");}
		vector<FeatureDescriptor * > * centers = new vector<FeatureDescriptor * >();
		for(int j = 0; j < nr_clusters; j++){centers->push_back(input.at(rand()%input.size())->clone());}
		float sum;
		printf("created clusters\n");
		for(int iter = 0; iter < iterations; iter++)
		{
			vector< vector<FeatureDescriptor * > * > * centers_data = new vector< vector <FeatureDescriptor * > * >();
			for(int j = 0; j < nr_clusters; j++){centers_data->push_back(new vector<FeatureDescriptor *>());}
			
			//printf("looking through input\n");
			sum = 0;
			for(unsigned int i = 0; i < input.size(); i++)
			{
				//printf("i:%i\n",i);
				FeatureDescriptor * current = input.at(i);
				float best = 99999;
				int best_id = -1;
				for(int j = 0; j < nr_clusters; j++)
				{
					float dist = current->distance(centers->at(j));
					//printf("%i %i -> %f\n",i,j,dist);
					//centers->at(j)->print();
					if(best > dist){
						best = dist;
						best_id = j;
					}
				}
				//current->print();
				//printf("sum:%f\n",sum);
				//printf("Best: %f\n",best);
				//printf("best_id:%i\n",best_id);
				sum+=best*best;
				centers_data->at(best_id)->push_back(current);
			}
			//printf("done with inputs...\n");
			sum /= float(input.size());
			if(verbose){printf("iter %i / %i errorsum: %f\n",iter+1,iterations,sum);}
			
			for(int j = 0; j < nr_clusters; j++)
			{
				if(centers_data->at(j)->size() > 0){
					centers->at(j)->update(centers_data->at(j));
				}
			}
			
			for(int j = 0; j < nr_clusters; j++){delete centers_data->back(); centers_data->pop_back();}
			delete centers_data;
		}
		
		printf("best_sum: %f\n",best_sum);
		sleep(1);
		if(best_sum == -1){
			best_centers = centers;
			best_sum = sum;
		}else if(sum < best_sum){
			best_centers = centers;
			best_sum = sum;
		}else{
			printf("time to delete\n");
			for(int i = 0; i < nr_clusters; i++){delete centers->at(i);}
			delete centers;
		}
	}
	return best_centers;
}
