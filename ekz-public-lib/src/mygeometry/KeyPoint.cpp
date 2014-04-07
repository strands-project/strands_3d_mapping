#include "KeyPoint.h"

KeyPoint::KeyPoint(){
	descriptor = 0;
	point = 0;
	r = 0;
	g = 0;
	b = 0;
	valid = false;
	chain = new KeyPointChain(this);
	frame_id = 0;
};
KeyPoint::~KeyPoint(){
	if(descriptor != 0){delete descriptor;}
	descriptor = 0;
	if(point != 0){delete point;}
	point = 0;
};

void KeyPoint::print(){
	printf("----------------------------------------------\n");
	for(unsigned int i = 0; i < cluster_distance_pairs.size();i++){
		printf("%i : %f\n",cluster_distance_pairs.at(i).first,cluster_distance_pairs.at(i).second);
	}
}

bool compare_v(pair <int , float > a, pair <int , float > b) {return (a.second < b.second);}

void KeyPoint::sortDistances(){std::sort (cluster_distance_pairs.begin(), cluster_distance_pairs.end(),compare_v);}

/*
struct value_index {
	float value;
	int index;
} value_index_object;

bool compare_v(value_index a,value_index b)
{
	return (a.value < b.value);
}

void KeyPoint::setWordDistances(float * distances, int nr_words){
	this->nr_words = nr_words;
	word_distances_value = distances;
	value_index * values = new value_index[nr_words]; 
	for(int i = 0; i < nr_words; i++)
	{
		values[i].value = distances[i];
		values[i].index = i;
	}
		
	std::vector< value_index > myvector (values, values+nr_words);
	std::sort (myvector.begin(), myvector.end(),compare_v);
	
	int counter = 0;
	vector<value_index>::iterator it;
	word_distances_sort_value = new float[nr_words];
	word_distances_sort_index = new int[nr_words];
	for (it=myvector.begin(); it!=myvector.end(); ++it){
		word_distances_sort_value[counter] = it->value;
		word_distances_sort_index[counter] = it->index;
		counter++;
	}
};
*/
