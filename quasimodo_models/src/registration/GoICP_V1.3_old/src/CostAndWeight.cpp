#include "CostAndWeight.h"
#include "math.h"

#define minvalue 0.000001

//#define useL1
#define useL2trunc

CostAndWeight::CostAndWeight()	{
	threshold = 4.0;
}

CostAndWeight::~CostAndWeight(){}

double CostAndWeight::getCost(double dist){
	#ifdef useL1
		return dist;
	#elif defined useL2trunc
		double squared_dist = dist*dist;
		if(squared_dist > threshold){	return threshold;}
		else{							return squared_dist;}
	#else 
		return dist*dist;
	#endif
}

double CostAndWeight::getCostSquaredDistance(double squared_dist){
	#ifdef useL1
		return sqrt(squared_dist);
	#elif defined useL2trunc
		if(squared_dist > threshold){	return threshold;}
		else{							return squared_dist;}
	#else 
		return squared_dist;
	#endif
}


double CostAndWeight::getWeightSquaredDistance(double squared_dist){
	#ifdef useL1
		return 1/sqrt(squared_dist+minvalue);
	#elif defined useL2trunc
		if(squared_dist > threshold){	return 0;}
		else{							return 1;}
	#else 
		return 1;
	#endif
}
