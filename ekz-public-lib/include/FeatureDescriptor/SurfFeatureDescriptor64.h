#ifndef SurfFeatureDescriptor64_H_
#define SurfFeatureDescriptor64_H_
#include "FeatureDescriptor.h"
using namespace std;
class SurfFeatureDescriptor64 : public FeatureDescriptor
{
	public:
	float stabilety;
	int laplacian;
	
	float * descriptor;
	unsigned int descriptor_length;
	
	SurfFeatureDescriptor64();
	SurfFeatureDescriptor64(float * feature_descriptor);
	SurfFeatureDescriptor64(float * feature_descriptor, int feature_laplacian);
	SurfFeatureDescriptor64(string path);
	double distance(SurfFeatureDescriptor64 * other_descriptor);
	void print();
	void store(string path);
	void update(vector<FeatureDescriptor * > * input);
	SurfFeatureDescriptor64 * clone();
	~SurfFeatureDescriptor64();
};
#endif
