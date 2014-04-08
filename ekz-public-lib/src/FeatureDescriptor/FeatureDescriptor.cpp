#include "FeatureDescriptor.h"
//#include "RGBFeatureDescriptor.h"
#include "SurfFeatureDescriptor64.h"
//#include "SurfFeatureDescriptor128.h"
#include "OrbFeatureDescriptor.h"
//#include "FPFHFeatureDescriptor.h"
//#include "IntegerHistogramFeatureDescriptor.h"
//#include "FloatHistogramFeatureDescriptor.h"

FeatureDescriptor::FeatureDescriptor(){type = unitiated;};

FeatureDescriptor::~FeatureDescriptor(){};

void FeatureDescriptor::print(){}

inline double FeatureDescriptor::distance(FeatureDescriptor * other_descriptor)
{
	//printf("inline double FeatureDescriptor::distance(FeatureDescriptor * other_descriptor)\n");
	if(type != other_descriptor->type){return 999999;}
	else
	{
		if(type == surf64){			return ((SurfFeatureDescriptor64 * )	(this))->distance((SurfFeatureDescriptor64 * 	)other_descriptor);}
		if(type == surf128){		return ((SurfFeatureDescriptor128 * )	(this))->distance((SurfFeatureDescriptor128 * 	)other_descriptor);}
		if(type == orb){			return ((OrbFeatureDescriptor * )		(this))->distance((OrbFeatureDescriptor * 		)other_descriptor);}
		//else if(type == FPFH){	return ((FPFHFeatureDescriptor * )(this))->distance((FPFHFeatureDescriptor * )other_descriptor);}
		//else if(type == rgb){		return ((RGBFeatureDescriptor * )(this))->distance((RGBFeatureDescriptor * )other_descriptor);}
		return -1;
	}
}

inline FeatureDescriptor * FeatureDescriptor::add(FeatureDescriptor * feature){return this;}

inline FeatureDescriptor * FeatureDescriptor::mul(float val){return this;}

inline FeatureDescriptor * FeatureDescriptor::clone(){
	if		(type == surf64)		{	return ((SurfFeatureDescriptor64 * 	)(this))->clone();}
	if		(type == surf128)		{	return ((SurfFeatureDescriptor128 * )(this))->clone();}
	if		(type == orb)			{	return ((OrbFeatureDescriptor * 	)(this))->clone();}
	//else if	(type == FPFH)		{	return ((FPFHFeatureDescriptor * 	)(this))->clone();}
	return 0;
}

inline void FeatureDescriptor::update(vector<FeatureDescriptor * > * input){printf("no rule for update...\n");}

inline void FeatureDescriptor::store(string path){printf("no rule for store...\n");}

void FeatureDescriptor::normalize(){}
