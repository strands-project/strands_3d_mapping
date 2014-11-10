#ifndef SurfFeatureDescriptor128_H_
#define SurfFeatureDescriptor128_H_
#include "FeatureDescriptor.h"
using namespace std;
class SurfFeatureDescriptor128 : public FeatureDescriptor
{
	public:
	float stabilety;
	int laplacian;
	
	float * descriptor;
	unsigned int descriptor_length;
	
	SurfFeatureDescriptor128();
	SurfFeatureDescriptor128(float * feature_descriptor);
	SurfFeatureDescriptor128(float * feature_descriptor, int feature_laplacian);
	SurfFeatureDescriptor128(string path);
	double distance(SurfFeatureDescriptor128 * other_descriptor);
	void print();
	void store(string path);
	void update(vector<FeatureDescriptor * > * input);
	SurfFeatureDescriptor128 * clone();
	~SurfFeatureDescriptor128();
};
#endif
