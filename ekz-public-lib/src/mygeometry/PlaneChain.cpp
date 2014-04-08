#include "PlaneChain.h"
#include <algorithm>
#include <vector>
int plane_chain_id_counter = 0;

PlaneChain::PlaneChain(Plane * kp){
	id = plane_chain_id_counter++;
	planes.push_back(kp);
	r = rand()%256;
	g = rand()%256;
	b = rand()%256;
};

PlaneChain::~PlaneChain(){};

void PlaneChain::merge(PlaneChain * chain){
	if(chain != this){
		for(unsigned int i = 0; i < chain->planes.size(); i++){
			planes.push_back(chain->planes.at(i));
			chain->planes.at(i)->chain = this;
		}
		printf("merge size: %i\n",(int)planes.size());
		delete chain;
	}
}
