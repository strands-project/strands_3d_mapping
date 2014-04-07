#include "KeyPointChain.h"
#include <algorithm>
#include <vector>

int kpc = 0;

KeyPointChain::KeyPointChain(KeyPoint * kp){
	id = kpc++;
	key_points.push_back(kp);
};

KeyPointChain::~KeyPointChain(){};

void KeyPointChain::merge(KeyPointChain * chain){
	for(unsigned int i = 0; i < chain->key_points.size(); i++){
		//printf("chain->key_points.at(i)->frame->id: %i \n",chain->key_points.at(i)->frame_id);
		key_points.push_back(chain->key_points.at(i));
		chain->key_points.at(i)->chain = this;
	}
	delete chain;
}
